#!/bin/bash
# Script for building radio driver for ROS2
# use this to gain permission to run the script for the first time: chmod +x scripts/build-radio.sh

# build GPS driver
colcon build --symlink-install --packages-select radio

# check if build is successful
if [ $? -eq 0 ]; then
    echo "build success..."
    source install/setup.bash
    ros2 run radio sender_udp --ros-args -p target_ip:='10.223.75.164'
else
    echo "build failed..."
    exit 1
fi
