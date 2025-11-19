#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity

class SpawnHusky(Node):
    def __init__(self):
        super().__init__('spawn_husky_local')

        # Path to YOUR Husky model
        husky_path = "/home/Riley/Desktop/Clearpath_Robotics_Husky_A200/model.sdf"

        if not os.path.exists(husky_path):
            raise FileNotFoundError(f"Husky SDF not found: {husky_path}")

        # Load the XML from file
        with open(husky_path, 'r') as f:
            self.sdf_xml = f.read()

        # Create service client
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.get_logger().info("Waiting for /spawn_entity service...")
        self.cli.wait_for_service()

        # Prepare request
        req = SpawnEntity.Request()
        req.name = "husky"
        req.xml = self.sdf_xml
        req.robot_namespace = "/"
        req.initial_pose.position.x = 0.0
        req.initial_pose.position.y = 0.0
        req.initial_pose.position.z = 0.05

        # Call service
        self.get_logger().info("Spawning Husky model...")
        future = self.cli.call_async(req)
        future.add_done_callback(self.spawn_done)

    def spawn_done(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"Husky spawn result: {result.status_message}")
        except Exception as e:
            self.get_logger().error(f"Spawn failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SpawnHusky()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

