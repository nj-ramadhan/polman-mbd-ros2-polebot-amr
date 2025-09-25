#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class UltrasonicTFBroadcaster(Node):
    def __init__(self):
        super().__init__('ultrasonic_tf_broadcaster')
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_tf)  # 10 Hz

        # 🟢 Définir les 4 capteurs : nom de frame + position
        self.ultrasonic_transforms = [
            {
                'child_frame_id': 'uls_left_front',
                'translation': ( 0.20,  0.15, 0.10),  # x, y, z
                'rotation':    ( 0.0,   0.0,  0.0, 1.0)
            },
            {
                'child_frame_id': 'uls_left_back',
                'translation': (-0.20,  0.15, 0.10),
                'rotation':    ( 0.0,   0.0,  0.0, 1.0)
            },
            {
                'child_frame_id': 'uls_right_front',
                'translation': ( 0.20, -0.15, 0.10),
                'rotation':    ( 0.0,   0.0,  0.0, 1.0)
            },
            {
                'child_frame_id': 'uls_right_back',
                'translation': (-0.20, -0.15, 0.10),
                'rotation':    ( 0.0,   0.0,  0.0, 1.0)
            }
        ]

    def broadcast_tf(self):
        now = self.get_clock().now().to_msg()

        for tf_def in self.ultrasonic_transforms:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'base_link'  # 🔁 Référence du robot
            t.child_frame_id = tf_def['child_frame_id']

            t.transform.translation.x = tf_def['translation'][0]
            t.transform.translation.y = tf_def['translation'][1]
            t.transform.translation.z = tf_def['translation'][2]

            t.transform.rotation.x = tf_def['rotation'][0]
            t.transform.rotation.y = tf_def['rotation'][1]
            t.transform.rotation.z = tf_def['rotation'][2]
            t.transform.rotation.w = tf_def['rotation'][3]

            self.broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = UltrasonicTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

