#!/bin/bash
#biulds and runs the IMU 
# Navigate to your ROS 2 workspace
cd /home/ws

# Build the workspace
echo "Building ROS 2 workspace..."
colcon build --packages-select bno055 --symlink-install

# Check if build is successful
if [ $? -eq 0 ]; then
    echo "IMU build success..."
else
    echo "IMU build failed..."
    exit 1
fi
