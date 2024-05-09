#!/bin/bash
source install/setup.bash
echo "Running Radar Slam node..."
ros2 run radar slam --ros-args --log-level debug
