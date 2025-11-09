import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    pkg_polebot_description = FindPackageShare('polebot_amr_description').find('polebot_amr_description')       
    robot_description = os.path.join(pkg_polebot_description, 'src', 'description', 'polebot_amr_description.sdf')
    trolley_description = os.path.join(pkg_polebot_description, 'src', 'description', 'trolley_description.sdf')

    # Pass 'package_path' as an argument to xacro
    robot_description_config = xacro.process_file(
        robot_description,
        mappings={'package_path': pkg_polebot_description}
    ).toxml()

    trolley_description_config = xacro.process_file(
        trolley_description,
        mappings={'package_path': pkg_polebot_description}
    ).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': trolley_description_config}]
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
            arguments=['-d', os.path.join(pkg_polebot_description, 'rviz', 'polebot_amr_sim.rviz')],
            name='rviz2',
            output='screen',
        )
    ])