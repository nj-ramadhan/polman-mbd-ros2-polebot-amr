# polebot_amr_bringup/launch/nav2_simulation_launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    bringup_dir = get_package_share_directory('polebot_amr_bringup')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    description_dir = get_package_share_directory('polebot_amr_description')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam = LaunchConfiguration('slam', default='False')
    rviz = LaunchConfiguration('rviz', default='False')
    headless = LaunchConfiguration('headless', default='False')

    # URDF
    xacro_file = os.path.join(description_dir, 'urdf', 'robot', 'polebot_amr.xacro')
    robot_description = {
        'robot_description': 
        xacro.process_file(xacro_file, mappings={'package_path': description_dir}).toxml()
    }

    # Paths to config files
    nav2_params = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')
    slam_params = os.path.join(bringup_dir, 'params', 'slam_params.yaml')
    rviz_config = os.path.join(bringup_dir, 'rviz', 'nav2.rviz')

    # ==== LAUNCH DESCRIPTION ====
    return LaunchDescription([
        # Use simulation time
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('slam', default_value='False'),
        DeclareLaunchArgument('rviz', default_value='False'),
        DeclareLaunchArgument('headless', default_value='False'),

        # Start Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'verbose': 'false', 'headless': headless}.items(),
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}]
        ),

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'polebot_amr',
                '-topic', '/robot_description',
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        ),

        # Joint state publisher (optional for wheels)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz'))  # only if RViz needed
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz)
        ),

        # Nav2 Bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam': slam,
                'params_file': slam_params if LaunchConfiguration('slam').perform(None) == 'True' else nav2_params,
                'map': '',  # ignored if slam=True
                'autostart': 'true'
            }.items()
        ),
    ])