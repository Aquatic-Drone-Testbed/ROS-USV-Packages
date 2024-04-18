#!/bin/bash
# To gain permission: chmod +x scripts/run-video.sh
source install/setup.bash
echo "Running video node..."
ros2 run video_stream_py video_publisher