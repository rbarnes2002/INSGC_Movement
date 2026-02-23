#!/usr/bin/env python3
"""
urgency_implemented.py

- Publishes to /model/robot1/cmd_vel (default) or topic you pass
- Subscribes to /robot1/human_task (std_msgs/String JSON payload)
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
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


@dataclass
class HumanTask:
    task_id: str
    urgency: int
    action: str
    duration: float


class UrgencyImplemented(Node):
    def __init__(self, cmd_topic, human_topic, hz, linear, angular, ack_pause_s):
        super().__init__("urgency_implemented")
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.sub = self.create_subscription(String, human_topic, self.on_human_task, 10)

        self.dt = 1.0 / hz
        self.linear = linear
        self.angular = angular
        self.ack_pause_s = ack_pause_s

        self.task_queue: List[HumanTask] = []
        self.running = True
        self.task_durations = {}

        print(f"cmd_vel topic: {cmd_topic}")
        print(f"human task topic: {human_topic}")
        print("Baseline loop: square -> zigzag -> repeat")
        print("Human actions: pause / forward / backward")
        print(f"ACK pause on interrupt receive: {self.ack_pause_s:.1f}s")

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
            print(f"Queued task {task.task_id} | urgency={task.urgency} | action={task.action} | duration={task.duration}s")
        except Exception as e:
            print(f"Failed to parse human task JSON: {e}. Raw: {msg.data}")

    # ---------- Motion helpers ----------
    def publish_twist(self, linear: float, angular: float):
        t = Twist()
        t.linear.x = linear
        t.angular.z = angular
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
        for _ in range(4):
            if not self.running: break
            self.drive_for(self.linear, 0.0, side_seconds)
            self.stop_robot(0.15)
            turn_time = (math.pi/2) / max(self.angular, 1e-6)
            self.drive_for(0.0, self.angular, turn_time)
            self.stop_robot(0.15)
        elapsed = time.time() - start_time
        self.task_durations["square"] = elapsed
        print(f"BASELINE TASK FINISH: square in {elapsed:.2f}s")

    def baseline_zigzag(self, reps=6, forward_seconds=2.0, turn_seconds=0.8):
        start_time = time.time()
        print("BASELINE TASK START: zigzag")
        sign = 1.0
        for _ in range(reps):
            if not self.running: break
            self.drive_for(self.linear, 0.0, forward_seconds)
            self.drive_for(0.0, sign * 0.6 * self.angular, turn_seconds)
            sign *= -1.0
            self.stop_robot(0.1)
        elapsed = time.time() - start_time
        self.task_durations["zigzag"] = elapsed
        print(f"BASELINE TASK FINISH: zigzag in {elapsed:.2f}s")

    # ---------- Execute human task ----------
    def execute_task(self, task: HumanTask):
        start_time = time.time()
        print(f"\nEXECUTING task {task.task_id} (urgency={task.urgency})")
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
            self.stop_robot()
            print()  # move past last queue line
            self.print_summary()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cmd-topic", default="/model/robot1/cmd_vel")
    parser.add_argument("--human-topic", default="/robot1/human_task")
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

