import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_polebot_description = FindPackageShare('polebot_amr_description').find('polebot_amr_description')
    xacro_file = os.path.join(pkg_polebot_description, 'src', 'description', 'polebot_amr_description.sdf')
    rviz_config = os.path.join(pkg_polebot_description, 'rviz', 'polebot_amr_sens.rviz')

    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={'package_path': pkg_polebot_description}
    ).toxml()

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

    autonics_lsc_lidar_node = Node(
        package='lsc_ros2_driver',
        executable='autonics_lsc_lidar',
        name='autonics_lidar',
        output='screen',
        parameters=[{
            'addr': '192.168.0.1',
            'port': 8000,
            'frame_id': 'lidar_link_corrected',
            'range_min': 0.05,
            'range_max': 25.0,
            'intensities': True
        }]
    )

    static_transform_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_to_corrected_tf',
        arguments=['0', '0', '0', '-1.5708', '0', '0', 'lidar_link', 'lidar_link_corrected']
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
        static_transform_lidar_node,

        rviz_node,
    ])