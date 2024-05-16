#!/bin/bash
# Script for building video streaming package

source install/setup.bash
# sudo apt install libopencv-dev -y
sudo apt install ros-humble-cv-bridge -y
pip install imageio-ffmpeg
pip install imageio

colcon build --symlink-install --packages-select video_stream_py

# Check if build is successful
if [ $? -eq 0 ]; then
    echo "Video build success..."
else
    echo "Video build failed..."
    exit 1
fi