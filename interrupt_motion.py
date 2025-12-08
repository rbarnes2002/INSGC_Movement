#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import threading

class MotionInterrupt(Node):
    def __init__(self):
        super().__init__('motion_interrupt')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Motion interrupt node started.')

        # Start the interrupt thread
        threading.Thread(target=self.run_interrupts, daemon=True).start()

    def run_interrupts(self):
        while rclpy.ok():
            time.sleep(10.0)  # wait 10 seconds before interrupting
            self.interrupt_motion()

    def interrupt_motion(self):
        self.get_logger().info('Interrupting motion...')
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        # publish stop command strongly for ~3 seconds
        start = time.time()
        while time.time() - start < 3.0:
            self.publisher.publish(twist)
            time.sleep(0.05)

        self.get_logger().info('Resuming...')

def main(args=None):
    rclpy.init(args=args)
    node = MotionInterrupt()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
