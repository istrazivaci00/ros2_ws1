import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    geo_share = get_package_share_directory('geo_demo')
    slam_share = get_package_share_directory('slam_toolbox')

    params = os.path.join(geo_share, 'config', 'slam', 'ceres_cholesky.yaml')

    sim = Node(
        package='geo_demo', executable='lidar_sim',
        name='lidar_sim', output='screen',
    )

    base_to_laser = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_to_laser',
        arguments=['--frame-id', 'base_link', '--child-frame-id', 'laser'],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_sync_launch.py')),
        launch_arguments={
            'slam_params_file': params,
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([sim, base_to_laser, slam])