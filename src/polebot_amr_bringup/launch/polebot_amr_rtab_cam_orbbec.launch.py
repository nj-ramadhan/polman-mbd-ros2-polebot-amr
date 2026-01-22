import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_polebot_description = FindPackageShare('polebot_amr_description').find('polebot_amr_description')
    xacro_file = os.path.join(pkg_polebot_description, 'src', 'description', 'polebot_amr_description.sdf')
    rviz_config = os.path.join(pkg_polebot_description, 'rviz', 'polebot_amr_rtab.rviz')

    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={'package_path': pkg_polebot_description}
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    orbbec_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('orbbec_camera'), 'launch', 'astra.launch.py')
        )
    )

    parameters = [{
        'frame_id': 'base_link',
        'use_sim_time': False,
        'subscribe_depth': True,
        'approx_sync': True,
        'queue_size': 100,
        'wait_for_transform': 0.5,
    }]

    remappings = [
        ('rgb/image', '/camera/color/image_raw'),
        ('depth/image', '/camera/depth/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
    ]

    rtab_odom_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='visual_odometry',
        output='screen',
        parameters=parameters,
        remappings=remappings + [('odom', '/odom')]
    )

    rtab_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=parameters + [{
            'Rtabmap/DetectionRate': '1.0', # Batasi 1Hz agar PC tidak berat
        }],
        remappings=remappings + [('odom', '/odom')],
        arguments=['--delete_db_on_start']
    )

    rtab_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=parameters,
        remappings=remappings + [('odom', '/odom')]
    )

    return LaunchDescription([
        orbbec_camera_launch,
        robot_state_node,
        joint_state_node,
        rviz_node,
        rtab_odom_node,
        rtab_slam_node,
        rtab_viz_node,
    ])