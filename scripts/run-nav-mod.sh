#!/bin/bash
source install/setup.bash
echo "Running Navigation node..."
# ros2 navigation_module nav_module --ros-args --log-level debug
ros2 run navigation_module nav_module
