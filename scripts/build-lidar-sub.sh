#!/bin/bash
# Script for building lidar subscriber for ROS2
colcon build --symlink-install --packages-select lidar

# check if build is successful
if [ $? -eq 0 ]; then
    echo "Lidar Subscriber build success..."
else
    echo "Lidar Subscriber build failed..."
    exit 1
fi
