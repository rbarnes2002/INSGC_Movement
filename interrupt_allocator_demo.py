#!/usr/bin/env python3
"""
interrupt_allocator_demo.py (keyboard interrupts + stabilization + event logging + safe-point deferral + geofence recovery)

What this node does:
- Publishes Twist to cmd_vel (default: /model/robot1/cmd_vel)
- Subscribes to /robot1/human_task (std_msgs/String JSON)
- Baseline loop: square -> zigzag -> repeat
- Human interrupts:
    * urgent: may preempt mid-motion
    * nonurgent: waits until safe point (end of baseline segment)
  Then: ACK pause (visible), execute action, resume baseline
- Event logging: publishes JSON to /robot1/task_events

Safety addition:
- Subscribes to /model/robot1/odom
- Applies a soft geofence (x/y bounds)
- Slows near edges, and if very near or out-of-bounds triggers recovery:
    stop -> rotate inward -> creep forward -> stop
  Logs safety_recovery_start/end + safety_stop events.
"""

import argparse
import json
import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


@dataclass
class HumanTask:
    task_id: str
    urgency: str     # "urgent" | "nonurgent"
    action: str      # "pause" | "forward" | "stabilize"


class InterruptAllocatorDemo(Node):
    def __init__(
        self,
        cmd_topic: str,
        human_topic: str,
        hz: float,
        linear: float,
        angular: float,
        ack_pause_s: float,
        human_pause_urgent_s: float,
        human_pause_nonurgent_s: float,
        human_move_urgent_s: float,
        human_move_nonurgent_s: float,
        # safety params
        x_min: float,
        x_max: float,
        y_min: float,
        y_max: float,
        edge_margin: float,
        enable_recovery: bool,
    ):
        super().__init__("interrupt_allocator_demo")

        # Publishers / Subscribers
        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        self.sub = self.create_subscription(String, human_topic, self.on_human_task, 10)

        # Event log publisher (for metrics collection)
        self.event_pub = self.create_publisher(String, "/robot1/task_events", 10)
        self.robot_id = "robot1"

        # Motion params
        self.dt = 1.0 / hz
        self.linear = float(linear)
        self.angular = float(angular)

        # Visible acknowledgement pause (shows robot received interrupt)
        self.ack_pause_s = float(ack_pause_s)

        # Durations used for the human action
        self.human_pause_urgent_s = float(human_pause_urgent_s)
        self.human_pause_nonurgent_s = float(human_pause_nonurgent_s)
        self.human_move_urgent_s = float(human_move_urgent_s)
        self.human_move_nonurgent_s = float(human_move_nonurgent_s)

        # State
        self.pending_urgent: Optional[HumanTask] = None
        self.pending_nonurgent: Optional[HumanTask] = None
        self.is_in_interrupt = False
        self.mission_started = False

        # ---------- Safety / Geofence ----------
        self.x_min, self.x_max = float(x_min), float(x_max)
        self.y_min, self.y_max = float(y_min), float(y_max)
        self.edge_margin = float(edge_margin)
        self.enable_recovery = bool(enable_recovery)

        self.last_odom: Optional[Odometry] = None
        self.odom_sub = self.create_subscription(
            Odometry,
            "/model/robot1/odom",
            self.on_odom,
            20
        )

        self.get_logger().info(f"cmd_vel topic: {cmd_topic}")
        self.get_logger().info(f"human task topic: {human_topic}")
        self.get_logger().info("Baseline loop: square -> zigzag -> repeat")
        self.get_logger().info("Human actions: pause / forward / stabilize")
        self.get_logger().info("Policy: urgent preempts immediately; nonurgent waits for safe points")
        self.get_logger().info(f"ACK pause on interrupt receive: {self.ack_pause_s:.1f}s")
        self.get_logger().info(
            f"Geofence: x in [{self.x_min}, {self.x_max}], y in [{self.y_min}, {self.y_max}], "
            f"margin={self.edge_margin}, recovery={'ON' if self.enable_recovery else 'OFF'}"
        )

    # ---------- Event logging ----------
    def publish_event(self, event: str, **fields) -> None:
        payload = {
            "ts_unix": float(time.time()),
            "robot": self.robot_id,
            "event": event,
            **fields,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.event_pub.publish(msg)

    # ---------- Odom + Safety ----------
    def on_odom(self, msg: Odometry) -> None:
        self.last_odom = msg

    def get_xy(self) -> Optional[Tuple[float, float]]:
        if self.last_odom is None:
            return None
        p = self.last_odom.pose.pose.position
        return (p.x, p.y)

    def out_of_bounds(self, x: float, y: float) -> bool:
        return (x < self.x_min) or (x > self.x_max) or (y < self.y_min) or (y > self.y_max)

    def near_edge(self, x: float, y: float) -> bool:
        return (
            (x < self.x_min + self.edge_margin) or
            (x > self.x_max - self.edge_margin) or
            (y < self.y_min + self.edge_margin) or
            (y > self.y_max - self.edge_margin)
        )

    def recovery_turn_direction(self, x: float, y: float) -> float:
        """
        Returns sign for angular z (+ => left, - => right) to bias inward.
        We choose a consistent bias based on the closest boundary.
        """
        dx_min = abs(x - self.x_min)
        dx_max = abs(self.x_max - x)
        dy_min = abs(y - self.y_min)
        dy_max = abs(self.y_max - y)

        d = min(dx_min, dx_max, dy_min, dy_max)

        if d == dx_max:   # near +x edge
            return +1.0
        if d == dx_min:   # near -x edge
            return -1.0
        if d == dy_max:   # near +y edge
            return -1.0
        return +1.0       # near -y edge

    def recover_inward(self, x: float, y: float, reason: str) -> None:
        """
        Recovery routine:
          stop -> rotate inward -> creep forward -> stop
        """
        if not self.enable_recovery:
            self.get_logger().warn(f"SAFETY STOP (recovery disabled): {reason} at x={x:.2f}, y={y:.2f}")
            self.publish_event("safety_stop", x=x, y=y, reason=reason)
            self.stop(0.8)
            return

        self.get_logger().warn(f"SAFETY RECOVERY ({reason}) at x={x:.2f}, y={y:.2f}")
        self.publish_event("safety_recovery_start", x=x, y=y, reason=reason)

        # 1) stop immediately
        self.stop(0.4)

        # 2) rotate inward
        turn_sign = self.recovery_turn_direction(x, y)
        self.drive_for_raw(0.0, turn_sign * 0.9 * self.angular, 1.0)
        self.stop(0.2)

        # 3) creep inward
        self.drive_for_raw(0.2 * self.linear, 0.0, 1.0)
        self.stop(0.3)

        self.publish_event("safety_recovery_end", x=x, y=y, reason=reason)

    def safety_filter(self, v: float, w: float):
        """
        Returns (v_safe, w_safe, should_recover, x, y, reason)

        - If out of bounds -> recover.
        - If very near edge -> recover preemptively.
        - Else if near edge -> slow down.
        """
        xy = self.get_xy()
        if xy is None:
            return v, w, False, None, None, ""

        x, y = xy

        if self.out_of_bounds(x, y):
            return 0.0, 0.0, True, x, y, "out_of_bounds"

        tight = 0.25 * self.edge_margin
        very_near = (
            (x < self.x_min + tight) or (x > self.x_max - tight) or
            (y < self.y_min + tight) or (y > self.y_max - tight)
        )
        if very_near:
            return 0.0, 0.0, True, x, y, "very_near_edge"

        if self.near_edge(x, y):
            return 0.3 * v, 0.5 * w, False, x, y, "near_edge"

        return v, w, False, x, y, ""

    # ---------- ROS callbacks ----------
    def on_human_task(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            task = HumanTask(
                task_id=str(data.get("task_id", "human_unknown")),
                urgency=str(data.get("urgency", "urgent")),
                action=str(data.get("action", "pause")),
            )

            if task.urgency == "urgent":
                self.pending_urgent = task
            else:
                self.pending_nonurgent = task

            self.get_logger().warn(
                f"Received HUMAN TASK: id={task.task_id} urgency={task.urgency} action={task.action}"
            )

            self.publish_event(
                "interrupt_received",
                task_id=task.task_id,
                urgency=task.urgency,
                action=task.action,
            )

        except Exception as e:
            self.get_logger().error(f"Failed to parse human task JSON: {e}. Raw: {msg.data}")

    # ---------- Interrupt decision helper ----------
    def maybe_handle_interrupt(self, safe_point: bool) -> bool:
        if self.is_in_interrupt:
            return False

        if self.pending_urgent is not None:
            task = self.pending_urgent
            self.pending_urgent = None
            self.run_interrupt_task(task)
            return True

        if safe_point and (self.pending_nonurgent is not None):
            task = self.pending_nonurgent
            self.pending_nonurgent = None
            self.run_interrupt_task(task)
            return True

        return False

    # ---------- Motion helpers ----------
    def publish_twist(self, v: float, w: float) -> None:
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.pub.publish(t)

    def stop(self, seconds: float = 0.2) -> None:
        end = time.time() + seconds
        while time.time() < end:
            self.publish_twist(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(self.dt)

    def drive_for_raw(self, v: float, w: float, seconds: float) -> None:
        """
        Raw drive that ignores interrupts and safety filter.
        Used ONLY inside recovery routine to avoid recursion.
        """
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.0)
            self.publish_twist(v, w)
            time.sleep(self.dt)

    def drive_for(self, v: float, w: float, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.0)

            # Only urgent tasks can preempt mid-motion
            if (not self.is_in_interrupt) and (self.pending_urgent is not None):
                return

            # ---------- SAFETY ----------
            v_safe, w_safe, do_recover, x, y, reason = self.safety_filter(v, w)
            if do_recover and (x is not None) and (y is not None):
                self.recover_inward(x, y, reason)
                return

            self.publish_twist(v_safe, w_safe)
            time.sleep(self.dt)

    def terrain_stabilize(self, seconds: float, base_v: float) -> None:
        start = time.time()
        ramp_s = min(1.0, max(0.2, seconds * 0.2))
        w = 0.25 * self.angular

        while time.time() - start < seconds:
            rclpy.spin_once(self, timeout_sec=0.0)

            elapsed = time.time() - start
            v = base_v * (elapsed / ramp_s) if elapsed < ramp_s else base_v
            omega = w if int(elapsed / 0.5) % 2 == 0 else -w

            # ---------- SAFETY ----------
            v_safe, w_safe, do_recover, x, y, reason = self.safety_filter(v, omega)
            if do_recover and (x is not None) and (y is not None):
                self.recover_inward(x, y, reason)
                return

            self.publish_twist(v_safe, w_safe)
            time.sleep(self.dt)

        self.stop(0.2)

    # ---------- Baseline tasks ----------
    def task_square(self, side_seconds: float = 3.0) -> None:
        self.get_logger().info("BASELINE TASK START: square")
        self.publish_event("baseline_task_start", task_name="square")

        for i in range(4):
            self.get_logger().info(f"  square {i+1}/4 forward")
            self.drive_for(self.linear, 0.0, side_seconds)

            if self.maybe_handle_interrupt(safe_point=True):
                return
            self.stop(0.15)

            angle = math.pi / 2.0
            turn_time = angle / max(self.angular, 1e-6)
            self.get_logger().info(f"  square {i+1}/4 turn")
            self.drive_for(0.0, self.angular, turn_time)

            if self.maybe_handle_interrupt(safe_point=True):
                return
            self.stop(0.15)

        self.get_logger().info("BASELINE TASK FINISH: square")
        self.publish_event("baseline_task_end", task_name="square")

    def task_zigzag(self, reps: int = 6, forward_seconds: float = 2.0, turn_seconds: float = 0.8) -> None:
        self.get_logger().info("BASELINE TASK START: zigzag")
        self.publish_event("baseline_task_start", task_name="zigzag")

        sign = 1.0
        for i in range(reps):
            self.get_logger().info(f"  zigzag {i+1}/{reps} forward")
            self.drive_for(self.linear, 0.0, forward_seconds)

            if self.maybe_handle_interrupt(safe_point=True):
                return

            self.get_logger().info(f"  zigzag {i+1}/{reps} turn")
            self.drive_for(0.0, sign * (0.6 * self.angular), turn_seconds)

            if self.maybe_handle_interrupt(safe_point=True):
                return

            sign *= -1.0
            self.stop(0.1)

        self.get_logger().info("BASELINE TASK FINISH: zigzag")
        self.publish_event("baseline_task_end", task_name="zigzag")

    # ---------- Human interrupt behavior ----------
    def run_interrupt_task(self, task: HumanTask) -> None:
        self.is_in_interrupt = True

        self.get_logger().warn(f"INTERRUPT EXECUTING: {task.task_id} -> ACK pause {self.ack_pause_s:.1f}s")
        self.stop(self.ack_pause_s)

        self.publish_event(
            "interrupt_action_start",
            task_id=task.task_id,
            urgency=task.urgency,
            action=task.action,
        )

        urgent = (task.urgency == "urgent")

        if task.action == "pause":
            dur = self.human_pause_urgent_s if urgent else self.human_pause_nonurgent_s
            self.get_logger().warn(f"HUMAN ACTION: PAUSE for {dur:.1f}s (urgency={task.urgency})")
            self.stop(dur)

        elif task.action == "forward":
            dur = self.human_move_urgent_s if urgent else self.human_move_nonurgent_s
            self.get_logger().warn(f"HUMAN ACTION: FORWARD for {dur:.1f}s (urgency={task.urgency})")
            self.drive_for(0.7 * self.linear, 0.0, dur)
            self.stop(0.2)

        elif task.action == "stabilize":
            dur = self.human_move_urgent_s if urgent else self.human_move_nonurgent_s
            self.get_logger().warn(f"HUMAN ACTION: STABILIZE for {dur:.1f}s (urgency={task.urgency})")
            base_v = 0.35 * self.linear if urgent else 0.25 * self.linear
            self.terrain_stabilize(dur, base_v)

        else:
            self.get_logger().error(f"Unknown human action: {task.action}. Defaulting to short pause.")
            self.stop(1.0)

        self.publish_event(
            "interrupt_action_end",
            task_id=task.task_id,
            urgency=task.urgency,
            action=task.action,
        )

        self.get_logger().warn(f"INTERRUPT FINISH: {task.task_id}")
        self.is_in_interrupt = False

    # ---------- Main loop ----------
    def run(self) -> None:
        try:
            if not self.mission_started:
                self.mission_started = True
                self.publish_event("mission_start")

            while rclpy.ok():
                if self.maybe_handle_interrupt(safe_point=True):
                    continue

                self.task_square()
                self.stop(0.5)

                if self.maybe_handle_interrupt(safe_point=True):
                    continue

                self.task_zigzag()
                self.stop(0.5)

        finally:
            self.publish_event("mission_end")
            self.stop(0.5)
            self.get_logger().info("Stopped. Exiting.")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--cmd-topic", default="/model/robot1/cmd_vel")
    p.add_argument("--human-topic", default="/robot1/human_task")
    p.add_argument("--hz", type=float, default=20.0)
    p.add_argument("--linear", type=float, default=0.4)
    p.add_argument("--angular", type=float, default=1.2)

    # ACK pause (show receipt)
    p.add_argument("--ack-pause-s", type=float, default=1.0)

    # human action durations (urgent vs nonurgent)
    p.add_argument("--human-pause-urgent-s", type=float, default=4.0)
    p.add_argument("--human-pause-nonurgent-s", type=float, default=2.0)
    p.add_argument("--human-move-urgent-s", type=float, default=3.0)
    p.add_argument("--human-move-nonurgent-s", type=float, default=1.5)

    # safety / geofence
    p.add_argument("--x-min", type=float, default=-8.0)
    p.add_argument("--x-max", type=float, default=8.0)
    p.add_argument("--y-min", type=float, default=-8.0)
    p.add_argument("--y-max", type=float, default=8.0)
    p.add_argument("--edge-margin", type=float, default=0.8)
    p.add_argument("--disable-recovery", action="store_true", help="If set, safety stops instead of recover.")

    args = p.parse_args()

    rclpy.init()
    node = InterruptAllocatorDemo(
        cmd_topic=args.cmd_topic,
        human_topic=args.human_topic,
        hz=args.hz,
        linear=args.linear,
        angular=args.angular,
        ack_pause_s=args.ack_pause_s,
        human_pause_urgent_s=args.human_pause_urgent_s,
        human_pause_nonurgent_s=args.human_pause_nonurgent_s,
        human_move_urgent_s=args.human_move_urgent_s,
        human_move_nonurgent_s=args.human_move_nonurgent_s,
        x_min=args.x_min,
        x_max=args.x_max,
        y_min=args.y_min,
        y_max=args.y_max,
        edge_margin=args.edge_margin,
        enable_recovery=(not args.disable_recovery),
    )
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

