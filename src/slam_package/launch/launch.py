from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,  # Change to True if using simulated time
                'mode': 'mapping',
                'map_update_interval': 1.0,
            }],
            remappings=[
                ('scan', '/radar_registered'),
                ('odom', '/radar_odom')
            ]
        ),
        # RViz node is optional and can be launched separately
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', 'install/your_slam_package/share/your_slam_package/rviz/slam_toolbox_config.rviz'],
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        )
    ])
