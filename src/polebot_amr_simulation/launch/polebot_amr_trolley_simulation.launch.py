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
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    polebot_sim_dir = get_package_share_directory('polebot_amr_simulation')
    polebot_desc_dir = get_package_share_directory('polebot_amr_description')
    launch_dir = os.path.join(polebot_sim_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_trolley_state_pub = LaunchConfiguration('use_trolley_state_pub')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')

    # --- 🎯 DISTINCT POSE ARGUMENTS (CRITICAL FIX) ---
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')

    trolley_x = LaunchConfiguration('trolley_x')
    trolley_y = LaunchConfiguration('trolley_y')
    trolley_z = LaunchConfiguration('trolley_z')
    trolley_yaw = LaunchConfiguration('trolley_yaw')

    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')
    trolley_name = LaunchConfiguration('trolley_name')
    trolley_sdf = LaunchConfiguration('trolley_sdf')

    rviz_config_file = LaunchConfiguration('rviz_config_file')

    # TF remappings (for namespace compatibility)
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # --- 📢 DECLARE LAUNCH ARGUMENTS ---
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator', default_value='True',
        description='Whether to start the simulator'
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub', default_value='True',
        description='Start robot state publisher'
    )

    declare_use_trolley_state_pub_cmd = DeclareLaunchArgument(
        'use_trolley_state_pub', default_value='True',
        description='Start trolley state publisher'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(polebot_desc_dir, 'rviz', 'polebot_amr_sim.rviz'),
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

    declare_use_trolley_state_pub_cmd = DeclareLaunchArgument(
        'use_trolley_state_pub',
        default_value='True',
        description='Whether to start the trolley state publisher',
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless', default_value='False', description='Whether to execute gzclient)'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(polebot_sim_dir, 'world', 'depot.sdf'),
        description='Full path to world model file to load',
    )

    # --- 📍 ROBOT POSE ARGS ---
    declare_robot_x_cmd = DeclareLaunchArgument('robot_x', default_value='-2.0')
    declare_robot_y_cmd = DeclareLaunchArgument('robot_y', default_value='0.0')
    declare_robot_z_cmd = DeclareLaunchArgument('robot_z', default_value='0.35')
    declare_robot_yaw_cmd = DeclareLaunchArgument('robot_yaw', default_value='0.0')

    # --- 📍 TROLLEY POSE ARGS (UNIQUE NAMES!) ---
    declare_trolley_x_cmd = DeclareLaunchArgument('trolley_x', default_value='-4.0')
    declare_trolley_y_cmd = DeclareLaunchArgument('trolley_y', default_value='0.0')
    declare_trolley_z_cmd = DeclareLaunchArgument('trolley_z', default_value='0.35')
    declare_trolley_yaw_cmd = DeclareLaunchArgument('trolley_yaw', default_value='0.0')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', default_value='polebot_amr', description='Robot entity name'
    )

    declare_trolley_name_cmd = DeclareLaunchArgument(
        'trolley_name', default_value='trolley', description='Trolley entity name'
    )

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(polebot_desc_dir, 'src', 'description', 'polebot_amr_description.sdf'),
        description='Full path to robot sdf file to spawn the robot in gazebo',
    )

    declare_trolley_sdf_cmd = DeclareLaunchArgument(
        'trolley_sdf',
        default_value=os.path.join(polebot_desc_dir, 'src', 'description', 'trolley_description.sdf'),
        description='Full path to robot sdf file to spawn the trolley in gazebo',
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(polebot_desc_dir, 'rviz', 'polebot_amr_sim.rviz'),
        description='Full path to RViz config file'
    )

    # --- 🤖 ROBOT STATE PUBLISHER ---
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        remappings=remappings,
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro', ' ', robot_sdf]),
                value_type=str
            )
        }],
    )

    # --- 🛒 TROLLEY STATE PUBLISHER (unique name + robot_description param) ---
    start_trolley_state_publisher_cmd = Node(
        condition=IfCondition(use_trolley_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='trolley_state_publisher',
        namespace=namespace,
        output='screen',
        remappings=remappings,
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro', ' ', trolley_sdf]),
                value_type=str
            )
        }],
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
    )

    # --- 🌍 WORLD PREPROCESSING ---
    world_sdf = tempfile.mktemp(prefix='polebot_', suffix='.sdf')
    world_sdf_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world])

    # --- 🌐 GAZEBO SERVER ---    
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                         'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s ', world_sdf]}.items(),
        condition=IfCondition(use_simulator))

    remove_temp_sdf_file = RegisterEventHandler(event_handler=OnShutdown(
        on_shutdown=[
            OpaqueFunction(function=lambda _: os.remove(world_sdf))
        ]))

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(polebot_sim_dir, 'world'))

    # --- 🖼️ GAZEBO CLIENT (GUI) ---
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch',
                         'gz_sim.launch.py')
        ),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
        launch_arguments={'gz_args': ['-v4 -g ']}.items(),
    )

    # --- 🧾 ENV SETUP ---
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(polebot_sim_dir, 'world')
    )

    # --- 🤖 SPAWN ROBOT ---
    gz_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'polebot_amr_spawn.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_simulator': use_simulator,
            'use_sim_time': use_sim_time,
            'robot_name': robot_name,
            'robot_sdf': robot_sdf,
            'x_pose': robot_x,
            'y_pose': robot_y,
            'z_pose': robot_z,
            'yaw': robot_yaw,
        }.items()
    )

    # --- 🛒 SPAWN TROLLEY ---
    gz_trolley = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'trolley_spawn.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_simulator': use_simulator,
            'use_sim_time': use_sim_time,
            'robot_name': trolley_name,
            'robot_sdf': trolley_sdf,
            'x_pose': trolley_x,
            'y_pose': trolley_y,
            'z_pose': trolley_z,
            'yaw': trolley_yaw,
        }.items()
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_trolley_state_pub_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)

    ld.add_action(declare_robot_x_cmd)
    ld.add_action(declare_robot_y_cmd)
    ld.add_action(declare_robot_z_cmd)
    ld.add_action(declare_robot_yaw_cmd)
    ld.add_action(declare_trolley_x_cmd)
    ld.add_action(declare_trolley_y_cmd)
    ld.add_action(declare_trolley_z_cmd)
    ld.add_action(declare_trolley_yaw_cmd)

    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_trolley_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_trolley_sdf_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # World & Gazebo
    ld.add_action(set_env_vars_resources)
    ld.add_action(world_sdf_xacro)
    ld.add_action(remove_temp_sdf_file)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)

    # Spawn & RSP
    ld.add_action(gz_robot)
    ld.add_action(gz_trolley)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_trolley_state_publisher_cmd)
    ld.add_action(rviz_cmd)

    return ld