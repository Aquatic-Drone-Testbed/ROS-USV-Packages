#!/bin/bash
source install/setup.bash
echo "Running Lidar Subscriber node..."
# ros2 run lidar lidar_sub --ros-args --log-level debug
ros2 run lidar lidar_sub
