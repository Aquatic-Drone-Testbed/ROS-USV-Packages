#!/bin/bash
# To gain permission: chmod +x scripts/run-radio-sender.sh
echo "Running radio_sender node..."
source install/setup.bash
ros2 run radio sender_udp --ros-args -p control_station_ip:='192.168.0.177' -p ros1_ip:='127.0.0.1'
