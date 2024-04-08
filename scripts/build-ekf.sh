#!/bin/bash
#colcon build --packages-select robot_localization
# Source the ROS 2 setup file
echo "Sourcing setup files..."
source /opt/ros/humble/setup.bash
source install/setup.bash
# Runs the EKF node using the specified YAML configuration
ros2 run robot_localization ekf_node --ros-args --params-file /home/ws/src/robot_localization/params/ekf.yaml
