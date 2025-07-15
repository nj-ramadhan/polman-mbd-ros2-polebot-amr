from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command

def generate_launch_description():
    slam_params = os.path.join(
        get_package_share_directory('polebot_amr_slam'),
        'config',
        'mapper_params_online_sync.yaml'
    )
    # Chemins
    polebot_amr_description_path = get_package_share_directory('polebot_amr_description')
    polebot_amr_slam_path = get_package_share_directory('polebot_amr_slam')
    xacro_file = os.path.join(polebot_amr_description_path, 'urdf', 'robot', 'main_robot.xacro')
    rviz_config = os.path.join(polebot_amr_slam_path, 'rviz', 'slam.rviz')

    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_sync_launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': slam_params
        }.items()
    )

    autonics_lsc_lidar_node = Node(
        package='lsc_ros2_driver',
        executable='autonics_lsc_lidar',
        name='autonics_lidar',
        output='screen',
        parameters=[{
            'addr': '192.168.0.1',
            'port': 8000,
            'frame_id': 'lidar',
            'range_min': 0.05,
            'range_max': 25.0,
            'intensities': True
        }]
    )

    # Noeuds
    robot_state_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command(["xacro ", xacro_file]),
            "publish_fixed_joints": True
        }]
    )

    #lidar_tf_node = Node(
    #    executable='static_transform_publisher',
    #    name='lidar_static_tf',
    #    output='screen',
    #    arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'lidar']
    #)

    joint_state_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen"
    )

    joint_state_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    fake_odom_node = Node(
        package='polebot_amr_bringup',
        executable='fake_odom_publisher.py',
        name='fake_odom_publisher',
        output='screen'
    )

    return LaunchDescription([
        declare_sim_time_arg,
        slam_toolbox_launch,
        autonics_lsc_lidar_node,
        joint_state_node,
        joint_state_gui_node,
        rviz_node,
        fake_odom_node,
        robot_state_node,
    ])
