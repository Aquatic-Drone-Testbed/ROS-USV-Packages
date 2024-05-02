#!/bin/bash
# To gain permission: chmod +x scripts/run-gps.sh
source install/setup.bash
echo "Running Radar Receiver node..."
# ros2 run radar quantum --ros-args --log-level debug -p host:="193.168.1.2"
ros2 run radar receiver --ros-args --log-level debug
# ros2 run radar slam