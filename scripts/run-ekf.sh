#!/bin/bash
# To gain permission: chmod +x scripts/run-ekf.sh
echo "Running EKF node..."
source /opt/ros/humble/setup.bash
source install/setup.bash
# Runs the EKF node using the specified YAML configuration
ros2 launch robot_localization dual_ekf_navsat_example.launch.py
