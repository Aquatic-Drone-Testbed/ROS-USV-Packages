#!/bin/bash
# To gain permission: chmod +x scripts/run-radio-receiver.sh
echo "Running radio server node..."
source install/setup.bash
ros2 run radio server --ros-args --log-level info
