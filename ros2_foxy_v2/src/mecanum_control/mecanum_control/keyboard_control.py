#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("Keyboard control started. Use WASD to move, Q to quit.")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def loop(self):
        move_bindings = {
            'w': (1.0, 0.0, 0.0),
            's': (-1.0, 0.0, 0.0),
            'a': (0.0, 1.0, 0.0),
            'd': (0.0, -1.0, 0.0),
        }

        twist = Twist()

        while rclpy.ok():
            key = self.get_key()
            if key == 'q':
                break
            elif key in move_bindings:
                x, y, z = move_bindings[key]
                twist.linear.x = x
                twist.linear.y = y
                twist.angular.z = z
                self.publisher_.publish(twist)
            else:
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    try:
        node.loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

