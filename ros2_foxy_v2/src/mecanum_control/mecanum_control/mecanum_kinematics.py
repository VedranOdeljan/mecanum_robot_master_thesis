#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import serial
import time

class MecanumKinematics(Node):
    def __init__(self):
        super().__init__('mecanum_kinematics')

        # ROS2 subscriber za cmd_vel
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.listener_callback, 10)

        # ROS2 publisher za wheel speeds (prema Teensyju)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'wheel_speeds', 10)

        # pokušaj otvoriti serijski port
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
            time.sleep(2)
            self.get_logger().info(" Serial port /dev/ttyACM0 otvoren")
        except serial.SerialException:
            self.get_logger().error(" Ne mogu otvoriti /dev/ttyACM0 — provjeri priključak.")
            self.ser = None

        # dimenzije robota
        self.L = 0.54
        self.W = 0.68
        self.R = 0.05

        # timer za čitanje Teensy izlaza
        self.create_timer(0.05, self.read_teensy_serial)  # 20 Hz

        # buffer za parcijalne linije iz serijskog porta
        self.serial_buffer = ""

    def listener_callback(self, msg):
        """Pretvara cmd_vel u brzine kotača i šalje na Teensy te ispisuje u terminal."""
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        L, W, R = self.L, self.W, self.R
        w1 = (1/R) * (vx - vy - (L + W) * wz)
        w2 = (1/R) * (vx + vy + (L + W) * wz)
        w3 = (1/R) * (vx + vy - (L + W) * wz)
        w4 = (1/R) * (vx - vy + (L + W) * wz)

        speeds = Float32MultiArray()
        speeds.data = [float(w1), float(w2), float(w3), float(w4)]

        # publish na ROS2 topic
        self.publisher_.publish(speeds)

        # ispis brzina koje se šalju
        self.get_logger().info(f" Slanje brzina na Teensy (rad/s): {speeds.data}")

        # slanje na Teensy preko serijskog porta
        if self.ser and self.ser.is_open:
            try:
                line = ','.join(f"{x:.2f}" for x in speeds.data) + '\n'
                self.ser.write(line.encode('utf-8'))
            except serial.SerialException:
                self.get_logger().error(" Greška prilikom slanja podataka na Teensy")

    def read_teensy_serial(self):
        """Čita serijski izlaz s Teensyja (brzine s enkodera) i ispisuje ih."""
        if not (self.ser and self.ser.is_open):
            return

        try:
            incoming = self.ser.read(self.ser.in_waiting or 1).decode('utf-8', errors='ignore')
            if not incoming:
                return

            self.serial_buffer += incoming
            while '\n' in self.serial_buffer:
                line, self.serial_buffer = self.serial_buffer.split('\n', 1)
                line = line.strip()
                if not line:
                    continue

                try:
                    # očekuje 4 float vrijednosti odvojene zarezima
                    values = [float(x) for x in line.split(',')]
                    if len(values) == 4:
                        self.get_logger().info(f" Brzine s enkodera (rad/s): {values}")
                    else:
                        self.get_logger().warn(f"[WARN] Neispravan broj vrijednosti: {line}")
                except ValueError:
                    self.get_logger().warn(f"[WARN] Neispravan format: {line}")

        except serial.SerialException:
            self.get_logger().error(" Greška prilikom čitanja s Teensy — port možda zatvoren")

def main(args=None):
    rclpy.init(args=args)
    node = MecanumKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

