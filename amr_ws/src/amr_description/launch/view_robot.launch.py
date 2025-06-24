import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = '/home/amr/amr_ws/src/amr_description/amr.urdf'
    pkg_urdf_path = get_package_share_directory('amr_description')

    with open(urdf_file, 'r') as file:
        robot_description_config = file.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_gui': True}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_urdf_path, 'rviz', 'amr.rviz')],
            name='rviz2',
            output='screen',
        )
    ])
