#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from functools import partial

class SpawnHuskies(Node):
    def __init__(self):
        super().__init__('spawn_husky_local')

        # Path to YOUR Husky model SDF
        husky_path = "/home/Riley/Desktop/Clearpath_Robotics_Husky_A200/model.sdf"
        # Example for your other machine:
        # husky_path = "/home/ryan/Desktop/Clearpath_Robotics_Husky_A200/model.sdf"

        if not os.path.exists(husky_path):
            raise FileNotFoundError(f"Husky SDF not found: {husky_path}")

        # Load the XML from file once
        with open(husky_path, 'r') as f:
            self.sdf_xml = f.read()

        # Create service client
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.get_logger().info("Waiting for /spawn_entity service...")
        self.cli.wait_for_service()

        # Define two spawn configurations
        self.robots = [
            {
                "name": "husky1",
                "x": 0.0,
                "y": 0.0,
                "z": 0.05,
            },
            {
                "name": "husky2",
                "x": 2.0,   # a bit offset so they don't overlap
                "y": 0.0,
                "z": 0.05,
            },
        ]

        # Spawn each husky
        for robot in self.robots:
            self.spawn_one(robot["name"], robot["x"], robot["y"], robot["z"])

    def spawn_one(self, name, x, y, z):
        req = SpawnEntity.Request()
        req.name = name
        req.xml = self.sdf_xml
        req.robot_namespace = f"/{name}"
        req.initial_pose.position.x = float(x)
        req.initial_pose.position.y = float(y)
        req.initial_pose.position.z = float(z)

        self.get_logger().info(f"Spawning {name} at x={x}, y={y}, z={z}...")
        future = self.cli.call_async(req)
        future.add_done_callback(partial(self.spawn_done, name=name))

    def spawn_done(self, future, name):
        try:
            result = future.result()
            self.get_logger().info(f"{name} spawn result: {result.status_message}")
        except Exception as e:
            self.get_logger().error(f"Spawn of {name} failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SpawnHuskies()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

