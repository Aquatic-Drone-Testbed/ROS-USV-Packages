#!/bin/bash
# To gain permission: chmod +x scripts/run-gps.sh
source install/setup.bash
echo "Running Radar Receiver node..."
ros2 run radar receiver --ros-args --log-level debug
