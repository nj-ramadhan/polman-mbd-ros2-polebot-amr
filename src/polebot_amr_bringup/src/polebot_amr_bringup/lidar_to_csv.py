#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import csv
import time

class ScanSaver(Node):
    def __init__(self):
        super().__init__('scan_saver')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.callback, 10)
        self.file = open(f'lsc_scan_{int(time.time())}.csv', 'w')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['timestamp_sec', 'angle_min', 'angle_max', 'angle_increment'] + 
                             [f'range_{i}' for i in range(360)])  # assuming 360 points

    def callback(self, msg):
        row = [msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
               msg.angle_min, msg.angle_max, msg.angle_increment] + list(msg.ranges)
        self.writer.writerow(row)
        self.get_logger().info(f'Saved scan with {len(msg.ranges)} points')

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ScanSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()