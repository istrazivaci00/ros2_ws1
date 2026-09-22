import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    geo_share = get_package_share_directory('geo_demo')
    slam_share = get_package_share_directory('slam_toolbox')

    default_params = os.path.join(geo_share, 'config', 'slam', 'ceres_cholesky.yaml')

    params_file = LaunchConfiguration('params_file')
    out_csv = LaunchConfiguration('out_csv')
    seed = LaunchConfiguration('seed')
    laps = LaunchConfiguration('laps')

    args = [
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('out_csv', default_value='~/ros2_ws/results/run.csv'),
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument('laps', default_value='3.0'),
    ]

    sim = Node(
        package='geo_demo', executable='lidar_sim',
        name='lidar_sim', output='screen',
        parameters=[{
            'seed': ParameterValue(seed, value_type=int),
            'laps': ParameterValue(laps, value_type=float),
        }],
    )

    base_to_laser = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_to_laser',
        arguments=['--frame-id', 'base_link', '--child-frame-id', 'laser'],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_sync_launch.py')),
        launch_arguments={'slam_params_file': params_file,
                          'use_sim_time': 'false'}.items(),
    )

    error = Node(
        package='geo_demo', executable='pose_error',
        name='pose_error', output='screen',
        parameters=[{'out_csv': out_csv}],
    )

    return LaunchDescription(args + [sim, base_to_laser, slam, error])