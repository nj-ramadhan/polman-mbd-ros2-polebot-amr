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
    joy_params = os.path.join(get_package_share_directory('polebot_amr_bringup'),'params','joystick_params.yaml')

    fake_odom_node = Node(
        package='polebot_amr_bringup',
        executable='fake_odom_publisher.py',
        name='fake_odom_publisher',
        output='screen'
    )

    roboteq_driver_launch_path = os.path.join(
        get_package_share_directory('roboteq_ros2_driver'),
        'launch',
        'roboteq_ros2_driver.launch.py'
    )

    roboteq_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(roboteq_driver_launch_path),
    )

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
    
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('polebot_amr_slam'),
                'launch',
                'polebot_slam.launch.py'
            )
        )
    )

    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        joy_node,
        teleop_node,
        fake_odom_node,
        slam_toolbox_launch,
        roboteq_driver_launch,

    ])
