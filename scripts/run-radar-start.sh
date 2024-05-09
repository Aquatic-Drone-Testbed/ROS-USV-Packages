#!/bin/bash
# To gain permission: chmod +x scripts/run-radar-start.sh
echo "Start radar scanning..."
source install/setup.bash
ros2 topic pub /radar_control std_msgs/String '{data: start_scan}' -1
echo "Start command sent."