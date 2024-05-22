#!/bin/bash
# To gain permission: chmod +x scripts/run-ekf.sh
echo "Running EKF node..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# for the navsat_transform_node.launch.py
ros2 run robot_localization navsat_transform_node --ros-args -r /imu:=/bno055/imu