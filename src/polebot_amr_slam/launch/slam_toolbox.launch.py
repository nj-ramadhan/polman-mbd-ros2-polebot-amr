from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('polebot_amr_slam'),
        'config',
        'mapper_params_online_sync.yaml'
    )
    
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[config_path],  # <-- ici on utilise ton YAML bien formé
        output='screen'
    )

    return LaunchDescription([
           slam_node,
    ])
