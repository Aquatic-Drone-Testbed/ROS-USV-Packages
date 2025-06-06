#!/bin/bash
# To gain permission: chmod +x scripts/run-gps.sh
source install/setup.bash
echo "Running GPS node..."
ros2 run gps_driver gps_node