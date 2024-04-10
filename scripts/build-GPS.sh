#!/bin/bash
# Script for building GPS driver for ROS2
# use this to gain permission to run the script for the first time: chmod +x scripts/build-GPS.sh

# build GPS driver
colcon build --symlink-install --packages-select gps_driver

# check if build is successful
if [ $? -eq 0 ]; then
    echo "build success..."
    source install/setup.bash
    echo "running GPS driver node..."
    ros2 run gps_driver gps_node
else
    echo "build failed..."
    exit 1
fi
