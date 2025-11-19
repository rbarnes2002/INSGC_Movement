#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random
import time

class RandomDrive(Node):
    def __init__(self):
        super().__init__('random_drive_husky')

        # Publisher sends velocity commands
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Random motion started. Press CTRL+C to stop.")

        # Movement ranges (tune these)
        self.min_speed = 0.5    # minimum forward speed (m/s)
        self.max_speed = 3.0    # maximum forward speed (m/s)
        self.min_turn = 0.2     # minimum turn speed (rad/s)
        self.max_turn = 2.0     # maximum turn speed (rad/s)
        self.min_duration = 1.5 # shortest action (seconds)
        self.max_duration = 6.0 # longest action (seconds)

        # Timer runs forever
        self.timer = self.create_timer(0.1, self.motion_loop)

        # Internal state
        self.action_end_time = 0.0
        self.current_twist = Twist()

        # Pick first action
        self.pick_new_action()

    def pick_new_action(self):
        action = random.choice(["forward", "turn_left", "turn_right"])
        duration = random.uniform(self.min_duration, self.max_duration)

        twist = Twist()

        if action == "forward":
            twist.linear.x = random.uniform(self.min_speed, self.max_speed)
            self.get_logger().info(f"FORWARD at {twist.linear.x:.2f} m/s for {duration:.1f} s")
        elif action == "turn_left":
            twist.angular.z = random.uniform(self.min_turn, self.max_turn)
            self.get_logger().info(f"LEFT TURN at {twist.angular.z:.2f} rad/s for {duration:.1f} s")
        elif action == "turn_right":
            twist.angular.z = -random.uniform(self.min_turn, self.max_turn)
            self.get_logger().info(f"RIGHT TURN at {abs(twist.angular.z):.2f} rad/s for {duration:.1f} s")

        self.current_twist = twist
        self.action_end_time = time.time() + duration

    def motion_loop(self):
        now = time.time()

        # Perform action
        if now < self.action_end_time:
            self.pub.publish(self.current_twist)
        else:
            # Stop robot for a short moment
            self.pub.publish(Twist())
            time.sleep(0.2)
            self.pick_new_action()

def main(args=None):
    rclpy.init(args=args)
    node = RandomDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Hard stop on exit
        stop = Twist()
        for _ in range(10):  # publish multiple times
            node.pub.publish(stop)
            time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

