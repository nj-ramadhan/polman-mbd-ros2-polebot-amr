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

    # Chemins
    polebot_amr_description_path = get_package_share_directory('polebot_amr_description')
    polebot_amr_bringup_path = get_package_share_directory('polebot_amr_bringup')
    xacro_file = os.path.join(polebot_amr_description_path, 'urdf', 'robot', 'main_robot.xacro')
    rviz_config = os.path.join(polebot_amr_bringup_path, 'rviz', 'robot.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    joy_params = os.path.join(get_package_share_directory('polebot_amr_bringup'),'config','joystick_ble.yaml')

    # Orbbec camera launch file path
    orbbec_camera_launch_path = os.path.join(
        get_package_share_directory('orbbec_camera'),
        'launch',
        'astra.launch.py'
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

    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'camera_link'],
        name='camera_link_to_optical_frame'
    )

    lidar_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        output='screen',
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

    # Include Orbbec camera launch file with arguments
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

    fake_odom_node = Node(
        package='polebot_amr_bringup',
        executable='cmd_vel.launch.py',
        name='fake_odom_publisher',
        output='screen'
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
                'slam_toolbox.launch.py'
            )
        )
    )

    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        robot_state_node,
        autonics_lsc_lidar_node,
        camera_node,
        camera_tf_node,
        orbbec_camera_launch,
        lidar_tf_node,
        joint_state_node,
        joint_state_gui_node,
        rviz_node,
        fake_odom_node,
        joy_node,
        teleop_node,

    ])
