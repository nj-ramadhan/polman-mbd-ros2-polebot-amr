import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_polebot_bringup = get_package_share_directory('polebot_amr_bringup')    
    joy_params = os.path.join(pkg_polebot_bringup,'config','polebot_amr_joystick_params.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
         )

    # pkg_roboteq = get_package_share_directory('roboteq_ros2_driver')

    # Config
    # roboteq_config = os.path.join(pkg_roboteq, 'config/roboteq',
    #     'roboteq.yaml')
    
    roboteq_config = os.path.join(pkg_polebot_bringup, 'config/roboteq',
        'polebot_amr_roboteq.yaml')

    # Nodes
    roboteq_driver_launch = Node(
        package='roboteq_ros2_driver',
        executable='roboteq_ros2_driver',
        name='roboteq_ros2_driver',
        output='screen',
        parameters=[roboteq_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        joy_node,
        teleop_node,
        roboteq_driver_launch,
    ])
