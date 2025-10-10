import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_description = FindPackageShare('polebot_amr_description').find('polebot_amr_description')
    pkg_gazebo_ros = get_package_share_directory('ros_gz')

    # Process URDF
    xacro_file = os.path.join(pkg_description, 'urdf', 'robot', 'polebot_amr.xacro')
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={
            'package_path': pkg_description,
            'robot_name': 'polebot_amr',
            'use_gazebo': 'true'
        }
    ).toxml()

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg_gazebo_ros), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'verbose': 'false'}.items(),
    )

    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-entity', 'polebot_amr',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_state_publisher,
        gazebo,
        spawn_entity,
    ])