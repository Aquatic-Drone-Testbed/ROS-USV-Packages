#!/bin/bash
# To gain permission: chmod +x scripts/run-radio-receiver.sh
# sudo tcpdump -i eth0 -n udp port 39000
echo "Running Radio server node..."
source install/setup.bash
ros2 run radio server --ros-args --log-level info
