import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Pastikan driver kamera Orbbec Astra berjalan
    # Pastikan paket 'orbbec_camera' sudah terinstal
    astra_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('orbbec_camera'), 'launch', 'astra.launch.py')
        )
    )

    # Parameter umum untuk sinkronisasi dan performa
    parameters = [{
        'frame_id': 'camera_link',
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

    return LaunchDescription([
        astra_camera_launch,

        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='visual_odometry',
            output='screen',
            parameters=parameters,
            remappings=remappings + [('odom', '/odom')]
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=parameters + [{
                'Rtabmap/DetectionRate': '1.0', # Batasi 1Hz agar PC tidak berat
            }],
            remappings=remappings + [('odom', '/odom')],
            arguments=['--delete_db_on_start']
        ),

        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=parameters,
            remappings=remappings + [('odom', '/odom')]
        ),
    ])