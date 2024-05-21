#!/bin/bash
# Script for building ros bridge for ROS2
colcon build --symlink-install --packages-select polar_to_ros1

# check if build is successful
if [ $? -eq 0 ]; then
    echo "Radio build success..."
else
    echo "Radio build failed..."
    exit 1
fi
