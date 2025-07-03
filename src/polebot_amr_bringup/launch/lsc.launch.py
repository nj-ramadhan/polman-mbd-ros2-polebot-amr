from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lsc_ros2_driver',
            executable='lsc_ros2_driver_node',
            name='autonics_lidar',
            output='screen',
            parameters=[{
                'addr': '192.168.0.1',     # adresse IP réelle de ton LiDAR
                'port': 8000,              # port réel
                'frame_id': 'lidar_link',  # nom défini dans ton Xacro
                'range_min': 0.05,
                'range_max': 25.0,
                'intensities': True
            }]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'lidar_link'],
        ),
    ])
