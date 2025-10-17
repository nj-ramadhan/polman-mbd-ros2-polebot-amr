# launch/polebot_amr_bringup.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directories
    pkg_polebot_description = get_package_share_directory('polebot_amr_description')
    pkg_polebot_bringup = get_package_share_directory('polebot_amr_bringup')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Config file path
    nav2_params_file = PathJoinSubstitution([
        pkg_polebot_description, 'config', 'polebot_amr_nav2_params.yaml'
    ])

    joy_params_file = PathJoinSubstitution([
        pkg_polebot_bringup, 'params', 'joystick_params.yaml'
    ])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[
            joy_params_file,
            {'use_sim_time': True},
            {'stamped': True}
        ],
        remappings=[
            ('cmd_vel', '/demo/cmd_vel')
        ],
        output='screen',
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[
            joy_params_file,
            {'use_sim_time': True},
            {'stamped': True}
        ],
        remappings=[
            ('cmd_vel', '/demo/cmd_vel')
        ],
        output='screen',
    )
    
    return LaunchDescription([
        # Enable use_sim_time globally
        SetParameter(name='use_sim_time', value=True),

        # 1. Robot simulation (Gazebo + RViz)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_polebot_description, 'launch', 'polebot_amr_display.launch.py'])
            )
        ),

        # 2. SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_slam_toolbox, 'launch', 'online_async_launch.py'])
            ),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),

        # 3. Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
            ),
            launch_arguments={'params_file': nav2_params_file}.items()
        ),

        # 4. Teleop (with stamped Twist
        joy_node,
        teleop_node,
    ])