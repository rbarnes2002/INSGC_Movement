#!/usr/bin/env python3

import subprocess
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

MODEL_NAME = "husky"

def get_pose():
    """Reads Husky pose from Gazebo using gz model."""
    try:
        out = subprocess.check_output(
            ["gz", "model", "-m", MODEL_NAME, "-p"],
            stderr=subprocess.STDOUT,
            text=True
        ).strip()
        parts = out.split()
        if len(parts) != 6:
            return None
        x, y, z, roll, pitch, yaw = map(float, parts)
        return x, y, z, yaw
    except:
        return None

class GoToPoint(Node):
    def __init__(self, target_x, target_y):
        super().__init__("go_to_point")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.target_x = target_x
        self.target_y = target_y

        self.get_logger().info(f"Target point: ({self.target_x}, {self.target_y})")

        # Control parameters
        self.linear_speed = 3.0
        self.angular_speed = 1.0
        self.distance_tolerance = 0.3   # stop when this close

        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        pose = get_pose()
        if pose is None:
            self.get_logger().warn("Could not read Husky pose.")
            return

        x, y, z, yaw = pose

        # Distance to target
        dx = self.target_x - x
        dy = self.target_y - y
        distance = math.sqrt(dx*dx + dy*dy)

        # Stop if close enough
        if distance < self.distance_tolerance:
            self.stop_robot()
            self.get_logger().info("Reached target point!")
            rclpy.shutdown()
            return

        # Compute heading angle
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - yaw)

        cmd = Twist()

        # Rotate toward target
        if abs(angle_error) > 0.1:
            cmd.angular.z = self.angular_speed * (1 if angle_error > 0 else -1)
        else:
            # Move forward when roughly facing target
            cmd.linear.x = self.linear_speed

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        stop = Twist()
        self.cmd_pub.publish(stop)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

def main():
    rclpy.init()

    # Choose a target coordinate here:
    target_x = 5.0
    target_y = 3.0

    node = GoToPoint(target_x, target_y)
    rclpy.spin(node)

if __name__ == "__main__":
    main()

