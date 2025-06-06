#!/bin/bash
source install/setup.bash
echo "Running Imu Handler node..."
# ros2 run imu_handler imu_handler --ros-args --log-level debug
ros2 run imu_handler imu_handler
