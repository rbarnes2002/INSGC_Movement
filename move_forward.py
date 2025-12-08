#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveForward(Node):
    def __init__(self):
        super().__init__('move_forward')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        timer_period = 0.1  # publish every 0.1 seconds
        self.timer = self.create_timer(timer_period, self.move_straight)
        self.get_logger().info("Moving forward... (Ctrl+C to stop)")

    def move_straight(self):
        msg = Twist()
        msg.linear.x = 0.5   # Forward speed (adjust as needed)
        msg.angular.z = 0.0  # No rotation
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MoveForward()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot on shutdown
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

