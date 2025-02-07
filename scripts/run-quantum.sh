#!/bin/bash
# To gain permission: chmod +x scripts/run-gps.sh
source install/setup.bash
echo "Running Radar Quantum node..."
#quantum host for direct to router: 0.0.0.0
#quantum host for pi DHCP: 10.224.0.2
# ros2 run radar quantum --ros-args -p host:="10.224.0.2" --log-level debug
ros2 run radar quantum --ros-args -p host:="10.42.0.1"
# ros2 run radar quantum --ros-args -p host:="0.0.0.0"
# ros2 topic pub radar_control std_msgs/String 'data: start_scan' -1