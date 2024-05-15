from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os

def generate_launch_description():
    config_ekf = os.path.join(get_package_share_directory("robot_localization"), 'params', 'ekf.yaml')
    config_navsat = os.path.join(get_package_share_directory("robot_localization"), 'params', 'navsat_transform.yaml')
    
    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_ekf],
        ),
        # launch_ros.actions.Node(
        #     package='robot_localization',
        #     executable='navsat_transform_node',
        #     name='navsat_transform_node',
        #     output='screen',
        #     parameters=[config_navsat],
        # ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_bno055',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'bno055'],
            output='screen'
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='bno055_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'bno055', 'base_link'],
            output='screen'
        ),
         launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_gps',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'gps'],
            output='screen'
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'gps', 'base_link'],
            output='screen'
        ),
        # launch_ros.actions.Node(
        #     package='gps_driver',
        #     executable='gps_node',
        #     name='gps_driver',
        #     output='screen'
        # ),
    ])

