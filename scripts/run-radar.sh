#!/bin/bash
# To gain permission: chmod +x scripts/run-gps.sh
source install/setup.bash
echo "Running Radar node..."
ros2 run radar quantum
# ros2 run radar receiver
# ros2 run radar slam