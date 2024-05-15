#!/bin/bash
# To gain permission: chmod +x scripts/run-radar-stop.sh
echo "Stop radar scanning..."
source install/setup.bash
ros2 topic pub /radar_control std_msgs/String '{data: stop_scan}' -1
echo "Stop command sent."
