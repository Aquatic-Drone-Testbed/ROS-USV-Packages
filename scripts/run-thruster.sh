#!/bin/bash
# To gain permission: chmod +x scripts/run-thruster.sh
echo "Running thruster_control node..."
source install/setup.bash
sudo pigpiod
ros2 run thruster_control thruster_control --ros-args --log-level info