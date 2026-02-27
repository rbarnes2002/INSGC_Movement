#!/usr/bin/env python3
"""
interrupt_allocator_demo.py

What this node does:
- Publishes Twist to cmd_vel (default: /model/{robot}/cmd_vel)
- Subscribes to /{robot}/human_task (std_msgs/String JSON)
- Baseline loop: square -> zigzag -> repeat
- Human interrupts:
    * urgent: may preempt mid-motion
    * nonurgent: waits until safe point (end of baseline segment)
  Then: ACK pause (visible), execute action, resume baseline
- Actions supported:
    * pause
    * forward
    * stabilize
    * move_to   (Trey location-based task)
- Event logging:
    * publishes JSON to /{robot}/task_events
    * optional: also to /team/task_events (for 1 CSV across all robots)
- Safety:
    * subscribes to /model/{robot}/odom
    * soft geofence (x/y bounds), slows near edge, and recovery if very near/out-of-bounds

Edits added for professor logger:
- interrupt_received now includes:
    created_unix (human generation timestamp passthrough)
    received_unix (robot receive timestamp)
- nonurgent deferral metric:
    publishes interrupt_deferral with deferred_ms when robot finally starts a deferred request
- interrupt_action_start/end now also include created_unix/received_unix passthrough
"""

import argparse
import json
import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple, Any, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def now() -> float:
    return float(time.time())


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def quat_to_yaw(q) -> float:
    # yaw from quaternion (x,y,z,w)
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class HumanTask:
    task_id: str
    urgency_class: str  # "urgent" | "nonurgent"
    action: str
    payload: Dict[str, Any]


class InterruptAllocatorDemo(Node):
    def __init__(
        self,
        robot: str,
        cmd_topic: str,
        human_topic: str,
        odom_topic: str,
        event_topic: str,
        publish_team_events: bool,
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

        self.robot_id = robot

        # Publishers / Subscribers
        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        self.sub = self.create_subscription(String, human_topic, self.on_human_task, 10)

        # Event publishers
        self.event_pub_robot = self.create_publisher(String, event_topic, 50)
        self.publish_team_events = bool(publish_team_events)
        self.event_pub_team = (
            self.create_publisher(String, "/team/task_events", 50) if self.publish_team_events else None
        )

        # Odom
        self.last_odom: Optional[Odometry] = None
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.on_odom, 20)

        # Motion params
        self.dt = 1.0 / float(hz)
        self.linear = float(linear)
        self.angular = float(angular)

        # Visible acknowledgement pause (shows robot received interrupt)
        self.ack_pause_s = float(ack_pause_s)

        # Default durations (if task doesn't provide duration_s)
        self.human_pause_urgent_s = float(human_pause_urgent_s)
        self.human_pause_nonurgent_s = float(human_pause_nonurgent_s)
        self.human_move_urgent_s = float(human_move_urgent_s)
        self.human_move_nonurgent_s = float(human_move_nonurgent_s)

        # State
        self.pending_urgent: Optional[HumanTask] = None
        self.pending_nonurgent: Optional[HumanTask] = None
        self.is_in_interrupt = False
        self.mission_started = False

        # ---- Metrics state for professor logger ----
        # When each human task was received by this robot (robot-side timestamp)
        self.received_unix_by_task: Dict[str, float] = {}
        # For nonurgent tasks, when we first started deferring (to compute deferred_ms)
        self.defer_start_unix_by_task: Dict[str, float] = {}

        # Safety / Geofence
        self.x_min, self.x_max = float(x_min), float(x_max)
        self.y_min, self.y_max = float(y_min), float(y_max)
        self.edge_margin = float(edge_margin)
        self.enable_recovery = bool(enable_recovery)

        self.get_logger().info(f"robot: {self.robot_id}")
        self.get_logger().info(f"cmd_vel topic: {cmd_topic}")
        self.get_logger().info(f"human task topic: {human_topic}")
        self.get_logger().info(f"odom topic: {odom_topic}")
        self.get_logger().info(f"event topic: {event_topic} (+ /team/task_events={self.publish_team_events})")
        self.get_logger().info("Baseline loop: square -> zigzag -> repeat")
        self.get_logger().info("Actions: pause / forward / stabilize / move_to")
        self.get_logger().info("Policy: urgent preempts immediately; nonurgent waits for safe points")
        self.get_logger().info(
            f"Geofence: x[{self.x_min},{self.x_max}] y[{self.y_min},{self.y_max}] margin={self.edge_margin} "
            f"recovery={'ON' if self.enable_recovery else 'OFF'}"
        )

    # ---------- Event logging ----------
    def publish_event(self, event: str, **fields) -> None:
        payload = {
            "ts_unix": now(),
            "robot": self.robot_id,
            "event": event,
            **fields,
        }
        msg = String()
        msg.data = json.dumps(payload)

        self.event_pub_robot.publish(msg)
        if self.publish_team_events and self.event_pub_team is not None:
            self.event_pub_team.publish(msg)

    # ---------- Odom + Pose ----------
    def on_odom(self, msg: Odometry) -> None:
        self.last_odom = msg

    def get_pose2d(self) -> Optional[Tuple[float, float, float]]:
        if self.last_odom is None:
            return None
        p = self.last_odom.pose.pose.position
        q = self.last_odom.pose.pose.orientation
        yaw = quat_to_yaw(q)
        return (p.x, p.y, yaw)

    # ---------- Safety / Geofence ----------
    def out_of_bounds(self, x: float, y: float) -> bool:
        return (x < self.x_min) or (x > self.x_max) or (y < self.y_min) or (y > self.y_max)

    def near_edge(self, x: float, y: float) -> bool:
        return (
            (x < self.x_min + self.edge_margin)
            or (x > self.x_max - self.edge_margin)
            or (y < self.y_min + self.edge_margin)
            or (y > self.y_max - self.edge_margin)
        )

    def recovery_turn_direction(self, x: float, y: float) -> float:
        dx_min = abs(x - self.x_min)
        dx_max = abs(self.x_max - x)
        dy_min = abs(y - self.y_min)
        dy_max = abs(self.y_max - y)
        d = min(dx_min, dx_max, dy_min, dy_max)

        if d == dx_max:  # near +x edge
            return +1.0
        if d == dx_min:  # near -x edge
            return -1.0
        if d == dy_max:  # near +y edge
            return -1.0
        return +1.0  # near -y edge

    def recover_inward(self, x: float, y: float, reason: str) -> None:
        if not self.enable_recovery:
            self.get_logger().warn(f"SAFETY STOP (recovery disabled): {reason} at x={x:.2f}, y={y:.2f}")
            self.publish_event("safety_stop", x=x, y=y, reason=reason)
            self.stop(0.8)
            return

        self.get_logger().warn(f"SAFETY RECOVERY ({reason}) at x={x:.2f}, y={y:.2f}")
        self.publish_event("safety_recovery_start", x=x, y=y, reason=reason)

        # stop immediately
        self.stop(0.4)

        # rotate inward
        turn_sign = self.recovery_turn_direction(x, y)
        self.drive_for_raw(0.0, turn_sign * 0.9 * self.angular, 1.0)
        self.stop(0.2)

        # creep inward
        self.drive_for_raw(0.2 * self.linear, 0.0, 1.0)
        self.stop(0.3)

        self.publish_event("safety_recovery_end", x=x, y=y, reason=reason)

    def safety_filter(self, v: float, w: float):
        pose = self.get_pose2d()
        if pose is None:
            return v, w, False, None, None, ""

        x, y, _ = pose

        if self.out_of_bounds(x, y):
            return 0.0, 0.0, True, x, y, "out_of_bounds"

        tight = 0.25 * self.edge_margin
        very_near = (
            (x < self.x_min + tight)
            or (x > self.x_max - tight)
            or (y < self.y_min + tight)
            or (y > self.y_max - tight)
        )
        if very_near:
            return 0.0, 0.0, True, x, y, "very_near_edge"

        if self.near_edge(x, y):
            return 0.3 * v, 0.5 * w, False, x, y, "near_edge"

        return v, w, False, x, y, ""

    # ---------- Interrupt parsing ----------
    def parse_urgency_class(self, data: Dict[str, Any]) -> Tuple[str, Optional[float]]:
        """
        Accepts urgency as:
          - "urgent"/"nonurgent"
          - numeric 0..10 (>=5 => urgent)
          - optional "urgency_class"
        Returns (urgency_class, urgency_num_or_none)
        """
        urgency_class = data.get("urgency_class", None)
        raw_urgency = data.get("urgency", None)
        urgency_num: Optional[float] = None

        if isinstance(urgency_class, str):
            u = urgency_class.strip().lower()
            if u in ("urgent", "nonurgent"):
                return u, None

        if isinstance(raw_urgency, str):
            u = raw_urgency.strip().lower()
            if u in ("urgent", "nonurgent"):
                return u, None
            try:
                urgency_num = float(u)
            except Exception:
                urgency_num = 0.0
        elif raw_urgency is None:
            urgency_num = 0.0
        else:
            try:
                urgency_num = float(raw_urgency)
            except Exception:
                urgency_num = 0.0

        urgency_class = "urgent" if float(urgency_num) >= 5.0 else "nonurgent"
        return urgency_class, urgency_num

    # ---------- ROS callback ----------
    def on_human_task(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)

            task_id = str(data.get("task_id", "human_unknown"))
            action = str(data.get("action", "pause"))

            # human interrupt generation time (from keyboard script)
            created_unix = data.get("created_unix", None)
            try:
                created_unix = float(created_unix) if created_unix is not None else None
            except Exception:
                created_unix = None

            # robot receive time
            received_unix = now()
            self.received_unix_by_task[task_id] = received_unix

            urgency_class, urgency_num = self.parse_urgency_class(data)

            task = HumanTask(
                task_id=task_id,
                urgency_class=urgency_class,
                action=action,
                payload=data,
            )

            if task.urgency_class == "urgent":
                self.pending_urgent = task
            else:
                self.pending_nonurgent = task

            if urgency_num is not None:
                self.get_logger().warn(
                    f"Received HUMAN TASK: id={task.task_id} urgency={urgency_num:.1f} ({task.urgency_class}) action={task.action}"
                )
            else:
                self.get_logger().warn(
                    f"Received HUMAN TASK: id={task.task_id} urgency={task.urgency_class} action={task.action}"
                )

            self.publish_event(
                "interrupt_received",
                task_id=task.task_id,
                urgency=task.urgency_class,
                urgency_num=urgency_num,
                action=task.action,
                # NEW for professor logger:
                created_unix=created_unix,
                received_unix=received_unix,
                deadline_type=data.get("deadline_type", None),
                deadline_s=data.get("deadline_s", None),
            )

        except Exception as e:
            self.get_logger().error(f"Failed to parse human task JSON: {e}. Raw: {msg.data}")

    # ---------- Interrupt decision helper ----------
    def maybe_handle_interrupt(self, safe_point: bool) -> bool:
        if self.is_in_interrupt:
            return False

        # Urgent: execute immediately
        if self.pending_urgent is not None:
            task = self.pending_urgent
            self.pending_urgent = None
            self.defer_start_unix_by_task.pop(task.task_id, None)
            self.run_interrupt_task(task)
            return True

        # Nonurgent: defer until safe_point
        if self.pending_nonurgent is not None:
            task = self.pending_nonurgent

            if not safe_point:
                if task.task_id not in self.defer_start_unix_by_task:
                    self.defer_start_unix_by_task[task.task_id] = now()
                return False

            # safe point reached -> execute now
            self.pending_nonurgent = None

            defer_start = self.defer_start_unix_by_task.pop(task.task_id, None)
            if defer_start is not None:
                deferred_ms = (now() - float(defer_start)) * 1000.0
                self.publish_event(
                    "interrupt_deferral",
                    task_id=task.task_id,
                    urgency=task.urgency_class,
                    action=task.action,
                    deferred_ms=deferred_ms,
                    created_unix=task.payload.get("created_unix", None),
                    received_unix=self.received_unix_by_task.get(task.task_id, None),
                )

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
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.0)
            self.publish_twist(v, w)
            time.sleep(self.dt)

    def drive_for(self, v: float, w: float, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.0)

            # urgent can preempt baseline mid-motion
            if (not self.is_in_interrupt) and (self.pending_urgent is not None):
                return

            # SAFETY
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

            # SAFETY
            v_safe, w_safe, do_recover, x, y, reason = self.safety_filter(v, omega)
            if do_recover and (x is not None) and (y is not None):
                self.recover_inward(x, y, reason)
                return

            self.publish_twist(v_safe, w_safe)
            time.sleep(self.dt)

        self.stop(0.2)

    # ---------- Trey-style location action ----------
    def move_to_xy(self, x_goal: float, y_goal: float, speed: float, tol: float, timeout_s: float = 60.0) -> None:
        start = time.time()

        k_w = 1.8
        w_max = 0.9 * self.angular
        v_max = max(0.05, min(abs(speed), self.linear))

        while rclpy.ok() and (time.time() - start) < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.0)

            pose = self.get_pose2d()
            if pose is None:
                self.publish_twist(0.0, 0.0)
                time.sleep(self.dt)
                continue

            x, y, yaw = pose
            dx = x_goal - x
            dy = y_goal - y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist <= tol:
                self.publish_twist(0.0, 0.0)
                self.stop(0.3)
                return

            heading = math.atan2(dy, dx)
            err = wrap_to_pi(heading - yaw)

            w = clamp(k_w * err, -w_max, w_max)

            facing = max(0.0, 1.0 - min(abs(err), math.pi) / math.pi)
            v = v_max * (0.25 + 0.75 * facing)
            v = min(v, 0.6 * dist)

            v_safe, w_safe, do_recover, sx, sy, reason = self.safety_filter(v, w)
            if do_recover and (sx is not None) and (sy is not None):
                self.recover_inward(sx, sy, reason)
                continue

            self.publish_twist(v_safe, w_safe)
            time.sleep(self.dt)

        self.publish_twist(0.0, 0.0)
        self.stop(0.2)
        self.get_logger().warn(f"move_to timed out after {timeout_s:.1f}s")

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
            urgency=task.urgency_class,
            action=task.action,
            # passthrough for professor logger
            created_unix=task.payload.get("created_unix", None),
            received_unix=self.received_unix_by_task.get(task.task_id, None),
        )

        urgent = (task.urgency_class == "urgent")
        data = task.payload

        duration_s = data.get("duration_s", None)
        if duration_s is not None:
            try:
                duration_s = float(duration_s)
            except Exception:
                duration_s = None

        if task.action == "pause":
            dur = (
                duration_s
                if duration_s is not None
                else (self.human_pause_urgent_s if urgent else self.human_pause_nonurgent_s)
            )
            self.get_logger().warn(f"HUMAN ACTION: PAUSE for {dur:.1f}s (urgency={task.urgency_class})")
            self.stop(dur)

        elif task.action == "forward":
            dur = (
                duration_s
                if duration_s is not None
                else (self.human_move_urgent_s if urgent else self.human_move_nonurgent_s)
            )
            self.get_logger().warn(f"HUMAN ACTION: FORWARD for {dur:.1f}s (urgency={task.urgency_class})")
            self.drive_for(0.7 * self.linear, 0.0, dur)
            self.stop(0.2)

        elif task.action == "backward":
            dur = (
                duration_s
                if duration_s is not None
                else (self.human_move_urgent_s if urgent else self.human_move_nonurgent_s)
            )
            self.get_logger().warn(f"HUMAN ACTION: BACKWARD for {dur:.1f}s (urgency={task.urgency_class})")
            self.drive_for(-0.5 * self.linear, 0.0, dur)
            self.stop(0.2)

        elif task.action == "stabilize":
            dur = (
                duration_s
                if duration_s is not None
                else (self.human_move_urgent_s if urgent else self.human_move_nonurgent_s)
            )
            self.get_logger().warn(f"HUMAN ACTION: STABILIZE for {dur:.1f}s (urgency={task.urgency_class})")
            base_v = (0.35 * self.linear) if urgent else (0.25 * self.linear)
            self.terrain_stabilize(dur, base_v)

        elif task.action == "move_to":
            x_goal = None
            y_goal = None

            if isinstance(data.get("target", None), dict):
                x_goal = data["target"].get("x", None)
                y_goal = data["target"].get("y", None)

            params = data.get("params", {}) or {}
            if x_goal is None:
                x_goal = params.get("x", None)
            if y_goal is None:
                y_goal = params.get("y", None)

            if x_goal is None or y_goal is None:
                self.get_logger().error("move_to missing target x/y (use target:{x,y} or params:{x,y})")
                self.stop(0.5)
            else:
                try:
                    x_goal = float(x_goal)
                    y_goal = float(y_goal)
                except Exception:
                    self.get_logger().error("move_to target x/y not numeric")
                    self.stop(0.5)
                else:
                    tol = float(data.get("tolerance", data.get("tolerance_m", 0.8)))
                    speed = float(data.get("speed", 0.6 * self.linear))
                    timeout_s = float(data.get("timeout_s", 60.0))
                    self.get_logger().warn(
                        f"HUMAN ACTION: MOVE_TO ({x_goal:.2f},{y_goal:.2f}) tol={tol:.2f} speed={speed:.2f}"
                    )
                    self.move_to_xy(x_goal, y_goal, speed=speed, tol=tol, timeout_s=timeout_s)

        else:
            self.get_logger().error(f"Unknown human action: {task.action}. Defaulting to short pause.")
            self.stop(1.0)

        self.publish_event(
            "interrupt_action_end",
            task_id=task.task_id,
            urgency=task.urgency_class,
            action=task.action,
            # passthrough for professor logger
            created_unix=task.payload.get("created_unix", None),
            received_unix=self.received_unix_by_task.get(task.task_id, None),
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

    p.add_argument("--robot", default="robot1", help="robot name, e.g., robot1/robot2/...")
    p.add_argument("--cmd-topic", default=None)
    p.add_argument("--human-topic", default=None)
    p.add_argument("--odom-topic", default=None)
    p.add_argument("--event-topic", default=None)
    p.add_argument("--publish-team-events", action="store_true", help="Also publish events to /team/task_events")

    p.add_argument("--hz", type=float, default=20.0)
    p.add_argument("--linear", type=float, default=0.4)
    p.add_argument("--angular", type=float, default=1.2)

    p.add_argument("--ack-pause-s", type=float, default=1.0)

    p.add_argument("--human-pause-urgent-s", type=float, default=4.0)
    p.add_argument("--human-pause-nonurgent-s", type=float, default=2.0)
    p.add_argument("--human-move-urgent-s", type=float, default=3.0)
    p.add_argument("--human-move-nonurgent-s", type=float, default=1.5)

    p.add_argument("--x-min", type=float, default=-8.0)
    p.add_argument("--x-max", type=float, default=8.0)
    p.add_argument("--y-min", type=float, default=-8.0)
    p.add_argument("--y-max", type=float, default=8.0)
    p.add_argument("--edge-margin", type=float, default=0.8)
    p.add_argument("--disable-recovery", action="store_true", help="If set, safety stops instead of recover.")

    args = p.parse_args()

    robot = str(args.robot)

    cmd_topic = args.cmd_topic or f"/model/{robot}/cmd_vel"
    human_topic = args.human_topic or f"/{robot}/human_task"
    odom_topic = args.odom_topic or f"/model/{robot}/odom"
    event_topic = args.event_topic or f"/{robot}/task_events"

    rclpy.init()
    node = InterruptAllocatorDemo(
        robot=robot,
        cmd_topic=cmd_topic,
        human_topic=human_topic,
        odom_topic=odom_topic,
        event_topic=event_topic,
        publish_team_events=args.publish_team_events,
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
