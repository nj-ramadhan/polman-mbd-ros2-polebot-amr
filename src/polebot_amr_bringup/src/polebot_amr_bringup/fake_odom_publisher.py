#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math


class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom_publisher')

        # Parameters
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('theta0', 0.0)

        rate = self.get_parameter('publish_rate').value
        self.x = self.get_parameter('x0').value
        self.y = self.get_parameter('y0').value
        self.theta = self.get_parameter('theta0').value

        # State
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        # Publishers & Broadcaster
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Timer
        self.timer = self.create_timer(1.0 / rate, self.publish_odom)

        self.get_logger().info("Fake odom publisher with teleop support started.")

    def cmd_vel_callback(self, msg):
        # Update commanded velocities
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = msg.angular.z

    def publish_odom(self):
        now = self.get_clock().now().to_msg()
        dt = 1.0 / 30.0  # assuming ~30 Hz

        # Integrate odometry (unicycle model)
        delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_th

        # Normalize theta
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # 1. Publish /odom
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth

        self.odom_pub.publish(odom)

        # 2. Publish odom → base_link TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = FakeOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()