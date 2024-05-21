#!/bin/bash
# To gain permission: chmod +x scripts/run-radio-sender.sh
echo "Running ros brige node..."
source install/setup.bash

# using default localhost
# ros2 run polar_to_ros1 polar_to_ros1_node
# using other ip
ros2 run polar_to_ros1 polar_to_ros1_node --ros-args -p host:="192.168.1.108"
