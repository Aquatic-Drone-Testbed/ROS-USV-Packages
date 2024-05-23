from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transform from map to odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        # Static transform from odom to base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),
        # Static transform from base_link to sensor (assuming sensor is the frame used by your radar)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_base_link_to_sensor',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'sensor'],
            output='screen'
        )
    ])
