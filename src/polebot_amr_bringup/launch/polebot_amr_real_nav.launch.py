import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    # --- Package paths ---
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_polebot_description = get_package_share_directory('polebot_amr_description')
    pkg_polebot_bringup = get_package_share_directory('polebot_amr_bringup')
    joy_params = os.path.join(pkg_polebot_bringup,'config','polebot_amr_joystick_params.yaml')

    # --- Launch configurations ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    slam = LaunchConfiguration('slam')
    use_namespace = LaunchConfiguration('use_namespace')
    namespace = LaunchConfiguration('namespace')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    # --- Declare launch arguments ---
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true. Set to false for real robot.'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to start RVIZ'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(pkg_polebot_description, 'rviz', 'polebot_amr_nav.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_nav2_bringup, 'maps', 'depot.yaml'),
        description='Full path to map file to load'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_polebot_bringup, 'config', 'polebot_amr_nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for Nav2'
    )

    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='True',
        description='Whether to run SLAM (true) or localization (false)'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace', default_value='false',
        description='Whether to apply a namespace to the navigation stack'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn crashed nodes (when composition is disabled)'
    )

    # --- Robot description (URDF/SDF) ---
    xacro_file = os.path.join(pkg_polebot_description, 'src', 'description', 'polebot_amr_description.sdf')
    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={'package_path': pkg_polebot_description}
    ).toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_config
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- Real Sensors ---
    # Autonics LSC LiDAR (publishes in native 'lidar_link' frame)
    autonics_lidar_node = Node(
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
            'intensities': True,
            'rate' : 5.0,
        }]
    )

    static_transform_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_to_corrected_tf',
        arguments=['0', '0', '0', '-1.5708', '0', '0', 'lidar_link', 'lidar_link_corrected']
    )

    # Orbbec Camera (Astra)
    orbbec_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('orbbec_camera'), 'launch', 'astra.launch.py')
        ),
        launch_arguments={
            'color_width': '640',
            'color_height': '480',
            'color_fps': '30',
            'depth_width': '640',
            'depth_height': '480',
            'depth_fps': '30',
            'use_sim_time': use_sim_time,
        }.items()
    )

    fake_odom_node = Node(
        package='polebot_amr_bringup',
        executable='fake_odom_publisher',
        name='fake_odom_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_rate': 5.0,
            # 'x0': -2.0,
            # 'y0': 0.5,
            # 'theta0': 0.0
        }]
    )

    # Replace fake_odom + robot_state_publisher with pure static TFs:
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    slam_params_file = os.path.join(
        pkg_polebot_bringup, 'config', 'polebot_amr_mapper_params.yaml'
    )

    start_slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
    )

    # --- Nav2 Navigation Stack ---
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
        }.items(),
    )

    # --- RViz Visualization ---
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'rviz_launch.py')
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'use_sim_time': use_sim_time,
            'rviz_config': rviz_config_file,
        }.items(),
    )

    roboteq_config = os.path.join(pkg_polebot_bringup, 'config/roboteq',
        'polebot_amr_roboteq.yaml')

    # Nodes
    roboteq_driver_launch = Node(
        package='roboteq_ros2_driver',
        executable='roboteq_ros2_driver',
        name='roboteq_ros2_driver',
        output='screen',
        parameters=[roboteq_config],
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
    
    # --- Assemble LaunchDescription ---
    ld = LaunchDescription()

    # Arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)

    # Robot state
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)

    # Sensors
    ld.add_action(autonics_lidar_node)
    ld.add_action(static_transform_lidar_node)
    # ld.add_action(orbbec_camera_launch)

    ld.add_action(fake_odom_node)
    ld.add_action(static_tf_map_odom)
    
    ld.add_action(start_slam_toolbox_node)

    # Navigation & Visualization
    ld.add_action(nav2_bringup_launch)
    ld.add_action(rviz_launch)

    ld.add_action(roboteq_driver_launch)
    ld.add_action(joy_node)
    ld.add_action(teleop_node)

    return ld