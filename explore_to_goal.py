#!/usr/bin/env python3
"""

Emilio Rodriguez

Standalone explorer: zig-zag path over the rectangle from robot start to goal.
Works for any robot (robot1, robot2, robot3, robot4, etc.).

Usage: python3 explore_to_goal.py <robot_name> <goal_x> <goal_y>
Examples:
  python3 explore_to_goal.py robot1 X-COORD Y-COORD
  python3 explore_to_goal.py robot2 225 60
  python3 explore_to_goal.py robot3 200 50
  python3 explore_to_goal.py robot4 200 50

- Gets start pose from Gazebo at launch
- Builds zig-zag waypoints (start = one corner, goal = opposite)
- Drives toward each vertex; returns when at goal or 4 min, then drives back to start
- Uses gz for pose; publishes to /model/<robot_name>/cmd_vel
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import subprocess
import re
import math
import time
import sys


EXPLORE_DURATION_SEC = 4 * 60
GOAL_TOLERANCE = 2.0
WAYPOINT_TOLERANCE = 1.2
MAX_LINEAR = 18.0
TURN_THRESHOLD = 0.4
TURN_LINEAR = 0.8
TURN_ANGULAR = 0.6
STEER_GAIN = 1.2
STEER_CAP = 0.7


def get_pose_gz(model_name):
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
            return (x, y, z, yaw, pitch)
        return None
    except Exception:
        return None


def build_zigzag_waypoints(start_x, start_y, goal_x, goal_y, max_vertices=20):
    """
    Build a zig-zag path: straight diagonal segments that touch top and
    bottom (or left and right) edges. Each vertex is a waypoint (subtask). Progress
    from start corner to goal corner along the longer axis; zig-zag along the other.
    """
    x_lo = min(start_x, goal_x)
    x_hi = max(start_x, goal_x)
    y_lo = min(start_y, goal_y)
    y_hi = max(start_y, goal_y)
    w = max(0.1, x_hi - x_lo)
    h = max(0.1, y_hi - y_lo)
    other_y = y_lo if start_y == y_hi else y_hi
    other_x = x_lo if start_x == x_hi else x_hi

    if w >= h:
        # Progress along X (start_x -> goal_x); zig between start_y and other_y
        n = max(2, min(max_vertices - 1, int(round(w / 8.0))))
        if n % 2 == 0:
            n += 1  # odd so last vertex is at goal (opposite y from start)
        waypoints = []
        for i in range(n + 1):
            t = i / n
            x = start_x + t * (goal_x - start_x)
            y = start_y if i % 2 == 0 else other_y
            waypoints.append((x, y))
        waypoints[-1] = (goal_x, goal_y)
        return waypoints
    else:
        # Progress along Y (start_y -> goal_y); zig between start_x and other_x
        n = max(2, min(max_vertices - 1, int(round(h / 8.0))))
        if n % 2 == 0:
            n += 1
        waypoints = []
        for i in range(n + 1):
            t = i / n
            y = start_y + t * (goal_y - start_y)
            x = start_x if i % 2 == 0 else other_x
            waypoints.append((x, y))
        waypoints[-1] = (goal_x, goal_y)
        return waypoints


class ExploreToGoalNode(Node):
    def __init__(self, robot_name, goal_x, goal_y):
        super().__init__("explore_to_goal", namespace=f"/{robot_name}")
        self.robot_name = robot_name
        self.goal_x = float(goal_x)
        self.goal_y = float(goal_y)
        self.start_x = None
        self.start_y = None
        self.waypoints = []
        self.phase = "exploring"  # exploring | returning | done
        self.start_time = time.time()
        self._return_pause_until = 0.0
        self.cmd_pub = self.create_publisher(Twist, f"/model/{robot_name}/cmd_vel", 10)
        self.control_timer = self.create_timer(0.08, self.control_step)  

        # Get start pose from Gazebo
        for _ in range(20):
            pose = get_pose_gz(robot_name)
            if pose is not None:
                self.start_x, self.start_y = pose[0], pose[1]
                self.get_logger().info(
                    "Start pose: (%.2f, %.2f); goal: (%.2f, %.2f)"
                    % (self.start_x, self.start_y, self.goal_x, self.goal_y)
                )
                break
            time.sleep(0.2)
        if self.start_x is None:
            self.get_logger().error("Could not get start pose from Gazebo")
            return

        self.waypoints = build_zigzag_waypoints(
            self.start_x, self.start_y, self.goal_x, self.goal_y, max_vertices=20
        )
        self._total_vertices = len(self.waypoints)
        self._logged_target = None
        self.get_logger().info(
            "Zig-zag path: %d vertices (subtasks) over rectangle to goal" % self._total_vertices
        )

    def control_step(self):
        if self.start_x is None or self.phase == "done":
            self.cmd_pub.publish(Twist())
            return

        pose = get_pose_gz(self.robot_name)
        if pose is None:
            self.cmd_pub.publish(Twist())
            return

        x, y, z, yaw, pitch = pose
        elapsed = time.time() - self.start_time

        # Check if goal has been reached
        dist_to_goal = math.hypot(self.goal_x - x, self.goal_y - y)
        if self.phase == "exploring" and dist_to_goal < GOAL_TOLERANCE:
            self.get_logger().info("Reached goal; returning to start.")
            self.phase = "returning"
            self.waypoints = [(self.start_x, self.start_y)]
            self._return_pause_until = time.time() + 1.0

        if self.phase == "exploring" and elapsed >= EXPLORE_DURATION_SEC:
            self.get_logger().info("4 min elapsed; returning to start.")
            self.phase = "returning"
            self.waypoints = [(self.start_x, self.start_y)]
            self._return_pause_until = time.time() + 1.0

        if self.phase == "returning" and time.time() < self._return_pause_until:
            self.cmd_pub.publish(Twist())
            return

        # If returning and reached start, stop
        if self.phase == "returning" and not self.waypoints:
            self.get_logger().info("Returned to start; done.")
            self.phase = "done"
            self.cmd_pub.publish(Twist())
            return

        # Current target: next waypoint or (start_x, start_y) when returning
        if self.phase == "returning":
            target_x, target_y = self.start_x, self.start_y
        elif self.waypoints:
            target_x, target_y = self.waypoints[0]
        else:
            self.cmd_pub.publish(Twist())
            return

        if (target_x, target_y) != self._logged_target:
            self._logged_target = (target_x, target_y)
            if self.phase == "returning":
                self.get_logger().info("Started subtask: return to start (%.1f, %.1f)" % (target_x, target_y))
            else:
                idx = self._total_vertices - len(self.waypoints) + 1
                self.get_logger().info("Started subtask %d: vertex (%.1f, %.1f)" % (idx, target_x, target_y))

        dx = target_x - x
        dy = target_y - y
        dist = math.hypot(dx, dy)

        # Check if reached waypoint
        if dist < WAYPOINT_TOLERANCE:
            if self.phase == "returning":
                self.phase = "done"
                self.get_logger().info("Reached start; done.")
                self.cmd_pub.publish(Twist())
                return
            if self.waypoints and self.phase == "exploring":
                vertex_idx = self._total_vertices - len(self.waypoints) + 1
                self.get_logger().info("Reached vertex %d (%.1f, %.1f)" % (vertex_idx, target_x, target_y))
                self.waypoints.pop(0)
            self.cmd_pub.publish(Twist())
            return

        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - yaw
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error <= -math.pi:
            angle_error += 2 * math.pi

        speed_scale = 0.85 if self.phase == "returning" else 1.0
        cmd = Twist()
        if abs(angle_error) > TURN_THRESHOLD:
            cmd.linear.x = TURN_LINEAR * speed_scale
            cmd.angular.z = (TURN_ANGULAR if angle_error > 0 else -TURN_ANGULAR) * speed_scale
        else:
            base_linear = min(MAX_LINEAR, 2.0 + (MAX_LINEAR - 2.0) * (1.0 - abs(angle_error) / TURN_THRESHOLD))
            cmd.linear.x = base_linear * speed_scale
            cmd.angular.z = max(-STEER_CAP, min(STEER_CAP, angle_error * STEER_GAIN)) * speed_scale
        self.cmd_pub.publish(cmd)


def main():
    if len(sys.argv) != 4:
        print("Usage: python3 explore_to_goal.py <robot_name> <goal_x> <goal_y>")
        print("  robot_name: robot1, robot2, robot3, robot4, etc.")
        print("Example: python3 explore_to_goal.py robot1 225 60")
        sys.exit(1)
    robot_name = sys.argv[1]
    goal_x = float(sys.argv[2])
    goal_y = float(sys.argv[3])

    rclpy.init()
    node = ExploreToGoalNode(robot_name, goal_x, goal_y)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
