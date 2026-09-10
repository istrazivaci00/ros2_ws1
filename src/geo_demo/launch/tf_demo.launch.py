from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
    )

    base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=[
            '--x', '0.20', '--y', '0.0', '--z', '0.15',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'laser',
        ],
    )

    odometry = Node(
        package='geo_demo',
        executable='fake_odometry',
        name='fake_odometry',
        output='screen',
    )

    return LaunchDescription([map_to_odom, base_to_laser, odometry])