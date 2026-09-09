import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('geo_demo')
    default_params = os.path.join(pkg_share, 'config', 'distance_params.yaml')

    params_file = LaunchConfiguration('params_file')

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Putanja do YAML fajla sa parametrima'
    )

    publisher = Node(
        package='geo_demo',
        executable='distance_publisher',
        name='distance_publisher',
        output='screen',
        parameters=[params_file],
    )

    subscriber = Node(
        package='geo_demo',
        executable='distance_subscriber',
        name='distance_subscriber',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([declare_params, publisher, subscriber])