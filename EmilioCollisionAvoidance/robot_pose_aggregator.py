#!/usr/bin/env python3
"""
robot_pose_aggregator.py

Publishes all robot poses and static obstacle poses (world frame) from Gazebo
to /robot_poses as a geometry_msgs/PoseArray for collision avoidance.

- Order: poses[0]=robot1, poses[1]=robot2, poses[2]=robot3, poses[3]=robot4,
  then poses[4]=first static obstacle (e.g. coop_umbrella), etc.
- Controllers treat indices 0..NUM_ROBOTS-1 as robots and NUM_ROBOTS.. as obstacles.
- Run this node alongside the sim and bridge; other nodes subscribe to /robot_poses.
"""

import math
import re
import subprocess
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Quaternion
from std_msgs.msg import Header


ROBOT_NAMES = ["robot1", "robot2", "robot3", "robot4"]
# Static obstacles (Gazebo model names) appended after robots in PoseArray.
# Rocks disabled to simplify sim; only coop and robot-robot avoidance used.
STATIC_OBSTACLES = [
    "coop_umbrella",
    # "falling_rock1",
    # "falling_rock2",
    # "falling_rock3",
    # "falling_rock4",
]
NUM_ROBOTS = len(ROBOT_NAMES)


def yaw_to_quaternion(yaw: float):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def get_pose_gz(model_name: str):
    """Query Gazebo for a model's pose. Returns (x, y, z, roll, pitch, yaw) or None."""
    try:
        out = subprocess.check_output(
            ["gz", "model", "-m", model_name, "-p"],
            stderr=subprocess.STDOUT,
            text=True,
            timeout=2,
        ).strip()
        parts = re.split(r"[ ,\[\]|]+", out)
        floats = []
        for p in parts:
            p = p.strip()
            if not p:
                continue
            try:
                floats.append(float(p))
            except ValueError:
                pass
        if len(floats) >= 6:
            x, y, z = floats[-6], floats[-5], floats[-4]
            roll, pitch, yaw = floats[-3], floats[-2], floats[-1]
            return (x, y, z, roll, pitch, yaw)
        return None
    except Exception:
        return None


class RobotPoseAggregator(Node):
    """Publish robot and obstacle poses as a PoseArray on /robot_poses."""

    def __init__(self, rate_hz: float = 20.0):
        super().__init__("robot_pose_aggregator")
        self.pub = self.create_publisher(PoseArray, "/robot_poses", 10)
        period = 1.0 / rate_hz if rate_hz > 0.0 else 0.05
        self.timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(
            f"Publishing poses: {ROBOT_NAMES} + obstacles {STATIC_OBSTACLES} on /robot_poses at {rate_hz} Hz"
        )

    def _on_timer(self):
        msg = PoseArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        # Robots
        for name in ROBOT_NAMES:
            p = get_pose_gz(name)
            pose = Pose()
            if p is not None:
                x, y, z, roll, pitch, yaw = p
                pose.position.x = x
                pose.position.y = y
                pose.position.z = z
                pose.orientation = yaw_to_quaternion(yaw)
            else:
                pose.position.x = 0.0
                pose.position.y = 0.0
                pose.position.z = 0.0
                pose.orientation.w = 1.0
            msg.poses.append(pose)

        # Static obstacles
        for name in STATIC_OBSTACLES:
            p = get_pose_gz(name)
            pose = Pose()
            if p is not None:
                x, y, z, roll, pitch, yaw = p
                pose.position.x = x
                pose.position.y = y
                pose.position.z = z
                pose.orientation.w = 1.0
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = 0.0
            else:
                pose.position.x = 0.0
                pose.position.y = 0.0
                pose.position.z = 0.0
                pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = RobotPoseAggregator(rate_hz=20.0)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
