import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    description_pkg_path = FindPackageShare('polebot_amr_description').find('polebot_amr_description')
    xacro_file = os.path.join(description_pkg_path, 'urdf', 'robot', 'polebot_amr.xacro')
    rviz_config = os.path.join(description_pkg_path, 'rviz', 'polebot_amr.rviz')

    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={'package_path': description_pkg_path}
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

        orbbec_camera_launch,

        rviz_node,
    ])