#!/bin/bash
# To gain permission: chmod +x scripts/run-gps.sh
source install/setup.bash
echo "Running Radar Quantum node..."
ros2 run radar quantum --ros-args --log-level info -p host:="10.224.0.2"
