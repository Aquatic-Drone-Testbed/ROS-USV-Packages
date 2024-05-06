from launch import LaunchDescription
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Define the directory where the parameters files are located.
    robot_localization_dir = get_package_share_directory('robot_localization')
    parameters_file_path = os.path.join(robot_localization_dir, 'params', 'dual_ekf_navsat_example.yaml')

    # Define the launch description.
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'output_final_position',
            default_value='false'),
        launch.actions.DeclareLaunchArgument(
            'output_location',
            default_value=os.path.join(robot_localization_dir, 'output', 'dual_ekf_navsat_example_debug.txt')),
        
        # Node for the EKF configured for odometry (local positioning).
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'dual_ekf_navsat_example.yaml')],
            remappings=[('odometry/filtered', 'odometry/local')]
        ),
        
        # Node for the EKF configured for map (global positioning).
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'dual_ekf_navsat_example.yaml')],
            remappings=[('odometry/filtered', 'odometry/global')]
        ),
        
        # Node for the NavSat Transform.
        launch_ros.actions.Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'dual_ekf_navsat_example.yaml')],
            remappings=[
                ('/bno055/imu', '/imu/data'),                       # IMU data topic
                ('/gps/fix', '/gps_data'),                   # GPS fix data topic
                ('/gps/filtered', '/filtered_gps'),         # Filtered GPS output topic
                ('/odometry/gps', '/gps/odom'),         # GPS odometry output
                ('/odometry/filtered', '/global_odometry')  # EKF global position output
            ]
        )
    ])
