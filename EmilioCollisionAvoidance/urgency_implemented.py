#!/usr/bin/env python3
"""
urgency_implemented.py

- Publishes to /model/robot1/cmd_vel (default) or topic you pass
- Subscribes to /robot1/human_task (std_msgs/String JSON payload)
- Optionally publishes to /robot1/task_events (JSON) for task_metrics_logger / data collection
- Baseline behavior: square -> zigzag -> repeat
- Human tasks preempt baseline based on numeric urgency (0-10)
- Task durations are tracked
- Live queue display added
- Ctrl+C safely stops the node and prints durations
"""

import argparse
import json
import math
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseArray


@dataclass
class HumanTask:
    task_id: str
    urgency: int
    action: str
    duration: float


# Order in /robot_poses PoseArray: robot1=0, robot2=1, robot3=2, robot4=3; then obstacles (e.g. coop_umbrella=4)
ROBOT_INDEX = {"robot1": 0, "robot2": 1, "robot3": 2, "robot4": 3}
NUM_ROBOTS = 4


class UrgencyImplemented(Node):
    def __init__(self, cmd_topic, human_topic, hz, linear, angular, ack_pause_s, events_topic, robot_name, poses_topic, avoidance_radius, obstacle_radius):
        super().__init__("urgency_implemented")
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.sub = self.create_subscription(String, human_topic, self.on_human_task, 10)
        self.events_topic = events_topic
        self.robot_name = robot_name
        self.event_pub = self.create_publisher(String, events_topic, 10) if events_topic else None

        self.dt = 1.0 / hz
        self.linear = linear
        self.angular = angular
        self.ack_pause_s = ack_pause_s
        self.avoidance_radius = avoidance_radius
        self.obstacle_radius = obstacle_radius
        self.robot_index = ROBOT_INDEX.get(robot_name, 0)
        self._poses_cache: Optional[PoseArray] = None
        self._poses_received = False
        self._last_no_poses_warn = 0.0
        if poses_topic:
            self._poses_sub = self.create_subscription(PoseArray, poses_topic, self._on_poses, 10)
        else:
            self._poses_sub = None

        self.task_queue: List[HumanTask] = []
        self.running = True
        self.task_durations = {}

        print(f"cmd_vel topic: {cmd_topic}")
        print(f"human task topic: {human_topic}")
        print("Baseline loop: square -> zigzag -> repeat")
        print("Human actions: pause / forward / backward")
        print(f"ACK pause on interrupt receive: {self.ack_pause_s:.1f}s")
        if self.event_pub:
            print(f"Event logging: {self.events_topic} (robot={self.robot_name})")
        if poses_topic:
            print(f"Collision avoidance: subscribed to {poses_topic}, robot radius={self.avoidance_radius}m, obstacle radius={self.obstacle_radius}m")

    def _on_poses(self, msg: PoseArray):
        self._poses_cache = msg
        self._poses_received = True

    def _avoidance_scale(self) -> float:
        """Return scale in [0, 1] for cmd_vel based on distance to other robots and obstacles. 0 = stop, 1 = full speed."""
        # When avoidance is enabled, do not move until we have pose data (avoids driving into obstacle before first /robot_poses)
        if self._poses_sub is not None and not self._poses_received:
            now = time.time()
            if now - self._last_no_poses_warn >= 5.0:
                print(f"[{self.robot_name}] Waiting for /robot_poses — is robot_pose_aggregator.py running? (Disable with --poses-topic '')")
                self._last_no_poses_warn = now
            return 0.0
        if self._poses_cache is None or len(self._poses_cache.poses) <= self.robot_index:
            return 1.0
        me = self._poses_cache.poses[self.robot_index].position
        scale = 1.0
        # Minimum scale when close to obstacles (coop) so robots can still creep; robot-robot still full stop
        OBSTACLE_MIN_SCALE = 0.3
        for i, pose in enumerate(self._poses_cache.poses):
            if i == self.robot_index:
                continue
            dx = pose.position.x - me.x
            dy = pose.position.y - me.y
            d = math.hypot(dx, dy)
            is_obstacle = i >= NUM_ROBOTS
            r = self.obstacle_radius if is_obstacle else self.avoidance_radius
            # Longer ramp for robot-robot so they slow earlier when approaching head-on
            ramp = 1.0 if not is_obstacle else 0.5
            if d <= r:
                contribution = OBSTACLE_MIN_SCALE if is_obstacle else 0.0
                scale = min(scale, contribution)
            elif d < r + ramp:
                contribution = (d - r) / ramp
                if is_obstacle:
                    contribution = max(contribution, OBSTACLE_MIN_SCALE)
                scale = min(scale, contribution)
        return scale

    def _publish_event(self, event: str, **kwargs):
        if not self.event_pub:
            return
        payload = {"ts_unix": time.time(), "robot": self.robot_name, "event": event, **kwargs}
        msg = String()
        msg.data = json.dumps(payload)
        try:
            self.event_pub.publish(msg)
        except rclpy._rclpy_pybind11.RCLError:
            pass

    # ---------- ROS callback ----------
    def on_human_task(self, msg: String):
        try:
            data = json.loads(msg.data)
            task_id = data.get("task_id") or f"task_{int(time.time()*1000)}"
            task = HumanTask(
                task_id=str(task_id),
                urgency=int(data.get("urgency", 5)),
                action=str(data.get("action", "pause")),
                duration=float(data.get("duration", 2.0)),
            )
            self.task_queue.append(task)
            # sort descending urgency for preemption
            self.task_queue.sort(key=lambda t: t.urgency, reverse=True)
            self._publish_event("interrupt_received", task_id=task.task_id, urgency=task.urgency, action=task.action, task_name="human_task")
            print(f"Queued task {task.task_id} | urgency={task.urgency} | action={task.action} | duration={task.duration}s")
        except Exception as e:
            print(f"Failed to parse human task JSON: {e}. Raw: {msg.data}")

    # ---------- Motion helpers ----------
    def publish_twist(self, linear: float, angular: float):
        scale = self._avoidance_scale()
        t = Twist()
        t.linear.x = linear * scale
        t.angular.z = angular * scale
        try:
            self.cmd_pub.publish(t)
        except rclpy._rclpy_pybind11.RCLError:
            # Publisher invalid; ignore during shutdown
            pass

    def stop_robot(self, seconds=0.2):
        end = time.time() + seconds
        while time.time() < end and self.running:
            self.publish_twist(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(self.dt)

    def drive_for(self, linear, angular, seconds):
        end = time.time() + seconds
        while time.time() < end and self.running:
            rclpy.spin_once(self, timeout_sec=0.0)
            self.publish_twist(linear, angular)
            time.sleep(self.dt)

    # ---------- Baseline tasks ----------
    def baseline_square(self, side_seconds=3.0):
        start_time = time.time()
        print("BASELINE TASK START: square")
        self._publish_event("baseline_task_start", task_name="square")
        for _ in range(4):
            if not self.running: break
            self.drive_for(self.linear, 0.0, side_seconds)
            self.stop_robot(0.15)
            turn_time = (math.pi/2) / max(self.angular, 1e-6)
            self.drive_for(0.0, self.angular, turn_time)
            self.stop_robot(0.15)
        elapsed = time.time() - start_time
        self.task_durations["square"] = elapsed
        self._publish_event("baseline_task_end", task_name="square")
        print(f"BASELINE TASK FINISH: square in {elapsed:.2f}s")

    def baseline_zigzag(self, reps=6, forward_seconds=2.0, turn_seconds=0.8):
        start_time = time.time()
        print("BASELINE TASK START: zigzag")
        self._publish_event("baseline_task_start", task_name="zigzag")
        sign = 1.0
        for _ in range(reps):
            if not self.running: break
            self.drive_for(self.linear, 0.0, forward_seconds)
            self.drive_for(0.0, sign * 0.6 * self.angular, turn_seconds)
            sign *= -1.0
            self.stop_robot(0.1)
        elapsed = time.time() - start_time
        self.task_durations["zigzag"] = elapsed
        self._publish_event("baseline_task_end", task_name="zigzag")
        print(f"BASELINE TASK FINISH: zigzag in {elapsed:.2f}s")

    # ---------- Execute human task ----------
    def execute_task(self, task: HumanTask):
        start_time = time.time()
        print(f"\nEXECUTING task {task.task_id} (urgency={task.urgency})")
        self._publish_event("interrupt_action_start", task_id=task.task_id, urgency=task.urgency, action=task.action, task_name="human_task")
        self.stop_robot(self.ack_pause_s)

        if task.action == "pause":
            self.stop_robot(task.duration)
        elif task.action == "forward":
            self.drive_for(self.linear, 0.0, task.duration)
            self.stop_robot(0.2)
        elif task.action == "backward":
            self.drive_for(-0.5*self.linear, 0.0, task.duration)
            self.stop_robot(0.2)
        else:
            self.stop_robot(1.0)

        elapsed = time.time() - start_time
        self.task_durations[task.task_id] = elapsed
        self._publish_event("interrupt_action_end", task_id=task.task_id, urgency=task.urgency, action=task.action, task_name="human_task")
        print(f"Task {task.task_id} completed in {elapsed:.2f}s")

    # ---------- Live queue display ----------
    def display_queue(self):
        if not self.task_queue:
            print("\rQueue: empty                               ", end="", flush=True)
            return
        queue_str = " | ".join(
            f"{t.task_id}(urg={t.urgency}, act={t.action}, dur={t.duration}s)"
            for t in self.task_queue
        )
        print(f"\rQueue: {queue_str}", end="", flush=True)

    # ---------- Summary ----------
    def print_summary(self):
        print("\nTask duration summary:")
        for tid, dur in self.task_durations.items():
            print(f"  {tid}: {dur:.2f}s")
        print("Node stopped cleanly")

    # ---------- Main loop ----------
    def run(self):
        try:
            self._publish_event("mission_start")
            last_display = 0
            while rclpy.ok() and self.running:
                if time.time() - last_display > 0.5:
                    self.display_queue()
                    last_display = time.time()

                if self.task_queue:
                    task = self.task_queue.pop(0)
                    self.execute_task(task)
                    continue

                # Baseline behavior
                self.baseline_square()
                self.baseline_zigzag()

        except KeyboardInterrupt:
            print("\nKeyboard interrupt received")
        finally:
            self.running = False
            self._publish_event("mission_end")
            self.stop_robot()
            print()  # move past last queue line
            self.print_summary()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cmd-topic", default="/model/robot1/cmd_vel")
    parser.add_argument("--human-topic", default="/robot1/human_task")
    parser.add_argument("--events-topic", default="/robot1/task_events", help="Publish task events for task_metrics_logger; use '' to disable")
    parser.add_argument("--robot-name", default="robot1", help="Robot name in event payloads")
    parser.add_argument("--poses-topic", default="", help="Subscribe to PoseArray for collision avoidance; pass /robot_poses to enable (requires robot_pose_aggregator.py)")
    parser.add_argument("--avoidance-radius", type=float, default=2.0, help="Stop when another robot is within this distance (m); larger = earlier stop to avoid head-on")
    parser.add_argument("--obstacle-radius", type=float, default=7.0, help="Stop when an obstacle (coop, rocks) is within this distance (m); use 7+ for falling rocks")
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--linear", type=float, default=0.4)
    parser.add_argument("--angular", type=float, default=1.2)
    parser.add_argument("--ack-pause-s", type=float, default=1.0)
    args = parser.parse_args()

    rclpy.init()
    node = UrgencyImplemented(
        cmd_topic=args.cmd_topic,
        human_topic=args.human_topic,
        hz=args.hz,
        linear=args.linear,
        angular=args.angular,
        ack_pause_s=args.ack_pause_s,
        events_topic=args.events_topic or None,
        robot_name=args.robot_name,
        poses_topic=args.poses_topic or None,
        avoidance_radius=args.avoidance_radius,
        obstacle_radius=args.obstacle_radius,
    )

    try:
        node.run()
    except KeyboardInterrupt:
        # Ctrl+C already handled inside run
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except rclpy._rclpy_pybind11.RCLError:
            pass  # ignore if already shutdown


if __name__ == "__main__":
    main()

