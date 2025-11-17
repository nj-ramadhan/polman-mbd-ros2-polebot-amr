import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_polebot_simulation = get_package_share_directory('polebot_amr_simulation')
    pkg_polebot_description = get_package_share_directory('polebot_amr_description')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_rtabmap_viz = LaunchConfiguration('use_rtabmap_viz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    pose = {
        'x': LaunchConfiguration('x_pose', default='0.00'),
        'y': LaunchConfiguration('y_pose', default='0.00'),
        'z': LaunchConfiguration('z_pose', default='0.35'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='0.00'),
    }
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')

    # Standard remappings
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(pkg_polebot_description, 'rviz', 'polebot_amr_nav.rviz'),
        description='Full path to the RVIZ config file to use',
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator',
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher',
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Whether to start RVIZ'
    )
    
    declare_use_rtabmap_viz_cmd = DeclareLaunchArgument(
        'use_rtabmap_viz',
        default_value='True',
        description='Whether to start RTAB-Map visualization',
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless', default_value='False', description='Whether to execute gzclient)'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_polebot_simulation, 'world', 'depot.sdf'),
        description='Full path to world model file to load',
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', default_value='polebot_amr', description='name of the robot'
    )

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(pkg_polebot_description, 'src', 'description', 'polebot_amr_description.sdf'),
        description='Full path to robot sdf file to spawn the robot in gazebo',
    )
    
    # --- RTAB-MAP NODES ---
    
    # Define the remappings for RTAB-Map to match your bridged topics
    rtabmap_remappings = [
        ('rgb/image', '/depth_camera/rgb/image_raw'),
        ('depth/image', '/depth_camera/depth/image_raw'),
        ('rgb/camera_info', '/depth_camera/camera_info'),
        ('scan', '/scan'),
        ('imu', '/imu'),
        ('odom', '/odom'), # Use odometry from your robot's DiffDrive plugin
    ]
    rtabmap_remappings.extend(remappings)

    # RTAB-Map SLAM node optimized for 3D mapping
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace=namespace,
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'use_sim_time': use_sim_time,
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': False, # <-- FIX: Set to False. RTAB-Map will still subscribe to /scan, but outside the sync.
            'subscribe_imu': False,  # <-- FIX: Set to False. We will rely on odom.
            'approx_sync': True,
            'approx_sync_odom': False, # <-- FIX: This is the key! Tells RTAB-Map NOT to put /odom in the sync group.
            'Reg/Force3DoF': 'false', # Allow full 6DoF mapping
            'Grid/FromDepth': 'true', # Create 3D grid from depth
        }],
        remappings=rtabmap_remappings,
    )
    
    # RTAB-Map Viz node
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=rtabmap_remappings,
        condition=IfCondition(use_rtabmap_viz),
    )
    
    # --- END RTAB-MAP NODES ---

    # --- SIMULATION NODES ---
    
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time, 'robot_description': Command(['xacro', ' ', robot_sdf])}
        ],
        remappings=remappings,
    )

    # Launch RVIZ directly
    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        namespace=namespace,
        output='screen',
    )

    # Process the world SDF file
    world_sdf = tempfile.mktemp(prefix='nav2_', suffix='.sdf')
    world_sdf_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world])
    
    # Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_sdf],
        output='screen',
        condition=IfCondition(use_simulator)
    )

    # Remove the temp file on shutdown
    remove_temp_sdf_file = RegisterEventHandler(event_handler=OnShutdown(
        on_shutdown=[
            OpaqueFunction(function=lambda _: os.remove(world_sdf))
        ]))

    # Set Gazebo resource path
    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_polebot_simulation, 'world'))
    
    # Gazebo client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch',
                         'gz_sim.launch.py')
        ),
        condition=IfCondition(PythonExpression(
            [use_simulator, ' and not ', headless])),
        launch_arguments={'gz_args': ['-v4 -g ']}.items(),
    )

    # Spawn the robot
    gz_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_polebot_simulation, 'launch', 'polebot_amr_spawn.launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_simulator': use_simulator,
                          'use_sim_time': use_sim_time,
                          'robot_name': robot_name,
                          'robot_sdf': robot_sdf,
                          'x_pose': pose['x'],
                          'y_pose': pose['y'],
                          'z_pose': pose['z'],
                          'roll': pose['R'],
                          'pitch': pose['P'],
                          'yaw': pose['Y']}.items())

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_rtabmap_viz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)

    # Add simulation actions
    ld.add_action(set_env_vars_resources)
    ld.add_action(world_sdf_xacro)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(gz_robot)
    ld.add_action(remove_temp_sdf_file)

    # Add core actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    
    # Add RTAB-Map actions
    ld.add_action(rtabmap_node)
    ld.add_action(rtabmap_viz_node)

    return ld