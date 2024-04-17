#!/bin/bash
# Script for running camera command

colcon build --symlink-install --packages-select video_stream_py

# Check if build is successful
if [ $? -eq 0 ]; then
    echo "Build success..."
    source install/setup.bash
    echo "Running camera publisher..."
    ros2 run video_stream_py video_publisher
else
    echo "Build failed..."
    exit 1
fi
