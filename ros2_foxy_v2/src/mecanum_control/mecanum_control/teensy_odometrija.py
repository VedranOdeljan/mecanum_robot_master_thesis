#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose, Point, Twist, Vector3
import math
import yaml

class MecanumOdometryNode(Node):
    def __init__(self):
        super().__init__('mecanum_odometry_node')

        # Pretplata na kutne brzine motora (rad/s)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'wheel_speeds',
            self.odometry_callback,
            10
        )

        # Publisher za odometriju
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Dimenzije robota i parametri kotača
        self.L = 0.54          # dužina robota [m]
        self.W = 0.68          # širina robota [m]
        self.r = 0.05          # polumjer kotača [m]
        self.gear_ratio = 5.0  # prijenosni omjer

        # Početna pozicija i orijentacija
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # yaw
        self.last_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

    def odometry_callback(self, msg: Float32MultiArray):
        if len(msg.data) != 4:
            self.get_logger().warn(f"[WARN] Neispravan broj motora: {msg.data}")
            return

        motor_w = [float(w) for w in msg.data]  # rad/s

        # Linearne brzine kotača (m/s)
        wheel_v = [w * self.r / self.gear_ratio for w in motor_w]

        # Mecanum kinematika (body frame)
        vx = (wheel_v[0] + wheel_v[1] + wheel_v[2] + wheel_v[3]) / 4.0
        vy = (-wheel_v[0] + wheel_v[1] + wheel_v[2] - wheel_v[3]) / 4.0
        wz = (-wheel_v[0] + wheel_v[1] - wheel_v[2] + wheel_v[3]) / (4.0 * (self.L/2 + self.W/2))

        # Vrijeme
        now = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        dt = now - self.last_time
        if dt <= 0:
            dt = 0.01
        self.last_time = now

        # Integracija pozicije (world frame)
        self.x += vx * math.cos(self.theta) * dt - vy * math.sin(self.theta) * dt
        self.y += vx * math.sin(self.theta) * dt + vy * math.cos(self.theta) * dt
        self.theta += wz * dt

        # Kreiranje Odometry poruke
        odom = Odometry()
        odom.header.stamp.sec = int(now)
        odom.header.stamp.nanosec = int((now - int(now)) * 1e9)
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Pose
        odom.pose.pose = Pose()
        odom.pose.pose.position = Point()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation = Quaternion()
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Twist
        odom.twist.twist = Twist()
        odom.twist.twist.linear = Vector3()
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular = Vector3()
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = wz

        # Publish
        self.odom_pub.publish(odom)

        # Ispis u terminal
        print(yaml.dump({
            "header": {"stamp": {"sec": odom.header.stamp.sec, "nanosec": odom.header.stamp.nanosec},
                       "frame_id": odom.header.frame_id},
            "child_frame_id": odom.child_frame_id,
            "pose": {"pose": {"position": {"x": self.x, "y": self.y, "z": 0.0},
                              "orientation": {"x": odom.pose.pose.orientation.x,
                                              "y": odom.pose.pose.orientation.y,
                                              "z": odom.pose.pose.orientation.z,
                                              "w": odom.pose.pose.orientation.w}}},
            "twist": {"twist": {"linear": {"x": vx, "y": vy, "z": 0.0},
                                "angular": {"x": 0.0, "y": 0.0, "z": wz}}}
        }, default_flow_style=False))

def main(args=None):
    rclpy.init(args=args)
    node = MecanumOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

