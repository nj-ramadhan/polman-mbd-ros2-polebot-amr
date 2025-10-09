from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    slam_params = os.path.join(
        get_package_share_directory('polebot_amr_slam'),
        'config',
        'mapper_params_online_sync.yaml'
    )
    # Chemins
    description_pkg_path = FindPackageShare('polebot_amr_description').find('polebot_amr_description')
    polebot_amr_slam_path = get_package_share_directory('polebot_amr_slam')
    xacro_file = os.path.join(description_pkg_path, 'urdf', 'robot', 'polebot_amr.xacro')
    rviz_config = os.path.join(polebot_amr_slam_path, 'rviz', 'slam.rviz')

    # Pass 'package_path' as an argument to xacro
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={'package_path': description_pkg_path}
    ).toxml()

    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    robot_state_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': robot_description_config}]
    )

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

    autonics_lsc_lidar_node = Node(
        package='lsc_ros2_driver',
        executable='autonics_lsc_lidar',
        name='autonics_lidar',
        output='screen',
        parameters=[{
            'addr': '192.168.0.1',
            'port': 8000,
            'frame_id': 'lidar_link',
            'range_min': 0.05,
            'range_max': 25.0,
            'intensities': True
        }]
    )
    
    orbbec_camera_launch_path = os.path.join(
        get_package_share_directory('orbbec_camera'),
        'launch',
        'astra.launch.py'
    )

    orbbec_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orbbec_camera_launch_path),
        launch_arguments={
            'color_width': '640',
            'color_height': '480',
            'color_fps': '30',
            'color_format': 'MJPG',
            'depth_width': '640',
            'depth_height': '480',
            'depth_fps': '30',
            'depth_format': 'Y11'
        }.items()
    )

    roboteq_driver_launch_path = os.path.join(
        get_package_share_directory('roboteq_ros2_driver'),
        'launch',
        'roboteq_ros2_driver.launch.py'
    )

    roboteq_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(roboteq_driver_launch_path),
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

    fake_odom_node = Node(
        package='polebot_amr_bringup',
        executable='fake_odom_publisher.py',
        name='fake_odom_publisher',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        # declare_sim_time_arg,
        robot_state_node,
        joint_state_node,

        autonics_lsc_lidar_node,

        orbbec_camera_launch,

        slam_toolbox_launch,
        rviz_node,
    ])
