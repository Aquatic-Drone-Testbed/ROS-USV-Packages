#!/bin/bash
# Script for building radio driver for ROS2
colcon build --symlink-install --packages-select radio

# check if build is successful
if [ $? -eq 0 ]; then
    echo "Radio build success..."
else
    echo "Radio build failed..."
    exit 1
fi
