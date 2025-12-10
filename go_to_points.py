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


class GoToPoints(Node):
    def __init__(self, points):
        super().__init__("go_to_points")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.points = points              # List of (x,y) tuples
        self.current_index = 0            # Start at first point

        self.target_x, self.target_y = self.points[self.current_index]
        self.get_logger().info(f"Going to first point: {self.target_x}, {self.target_y}")

        # Control parameters
        self.linear_speed = 3.0
        self.angular_speed = 1.0
        self.distance_tolerance = 0.3

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

        # If reached target, go to next point
        if distance < self.distance_tolerance:
            self.get_logger().info(f"Reached point ({self.target_x}, {self.target_y})")

            self.current_index += 1
            if self.current_index >= len(self.points):
                self.stop_robot()
                self.get_logger().info("All points reached! Shutting down.")
                rclpy.shutdown()
                return
            
            # Set next point
            self.target_x, self.target_y = self.points[self.current_index]
            self.get_logger().info(f"Now heading to next point: {self.target_x}, {self.target_y}")
            return

        # Compute steering
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - yaw)

        cmd = Twist()

        if abs(angle_error) > 0.1:
            cmd.angular.z = self.angular_speed * (1 if angle_error > 0 else -1)
        else:
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

    # Define a sequence of points: (x1,y1), then (x2,y2)
    point_A = (5.0, 3.0)
    point_B = (10.0, -2.0)
    #point_C = (-4.0, -4.0)
    #point_D = (-8.0, 7.0)

    node = GoToPoints([point_A, point_B]) #([point_A, point_B, point_C, point_D])
    rclpy.spin(node)


if __name__ == "__main__":
    main()

