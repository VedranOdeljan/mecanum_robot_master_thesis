#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import struct

class TeensyComm(Node):
    def __init__(self):
        super().__init__('teensy_comm')
        self.subscription = self.create_subscription(
            Float32MultiArray, 'wheel_speeds', self.listener_callback, 10)

        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.get_logger().info("Connected to Teensy 4.0 at /dev/ttyACM0")

    def listener_callback(self, msg):
        data = struct.pack('ffff', *msg.data)
        self.ser.write(data)

def main(args=None):
    rclpy.init(args=args)
    node = TeensyComm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

