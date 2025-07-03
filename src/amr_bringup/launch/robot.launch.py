import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():

    # Chemins
    amr_description_path = get_package_share_directory('amr_description')
    amr_bringup_path = get_package_share_directory('amr_bringup')
    xacro_file = os.path.join(amr_description_path, 'urdf', 'robot', 'main_robot.xacro')
    rviz_config = os.path.join(amr_bringup_path, 'rviz', 'amr.rviz')

    # Noeuds
    robot_state_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command(["xacro ", xacro_file])
        }]
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

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='usb_camera',
        parameters=[{
            'video_device': '/dev/video0',
            'frame_id': 'camera_link'
        }],
        output='screen'
    )

    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'lidar']
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        robot_state_node,
        autonics_lsc_lidar_node,
        camera_node,
        tf2_node,
        joint_state_node,
        joint_state_gui_node,
        rviz_node
    ])
