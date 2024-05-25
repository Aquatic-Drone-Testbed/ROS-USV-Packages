from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    slam_package_dir = get_package_share_directory('slam_package')
    static_transforms_launch_path = os.path.join(slam_package_dir, 'launch', 'static_transforms_launch.py')
    print(f"Using launch file: {static_transforms_launch_path}")
    return LaunchDescription([
        
        # DeclareLaunchArgument(
        #     'use_rviz',
        #     default_value='true',
        #     description='Whether to start RViz'
        # ),  
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_package_dir, 'launch', 'static_transforms_launch.py'))
        ),
        
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
            # remappings=[
            #     ('scan', '/ros2_radar_registered'),
            #     ('odom', '/ros2_radar_odom')
            # ]
        ),
        # RViz node is optional and can be launched separately
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     parameters=[os.path.join(slam_package_dir, 'config', 'qos_overrides.yaml'),
        #                 {'use_sim_time': True}],
        #     arguments=['-d', os.path.join(slam_package_dir, 'rviz', 'slam_toolbox_config.rviz')],
        #     condition=IfCondition(LaunchConfiguration('use_rviz'))
        # )
    ])
