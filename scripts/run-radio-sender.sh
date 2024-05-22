#!/bin/bash
# To gain permission: chmod +x scripts/run-radio-sender.sh
echo "Running radio_sender node..."
source install/setup.bash
ros2 run radio sender_udp --ros-args -p control_station_ip:='10.223.75.164' -p ros1_ip:='127.0.0.1' --log-level warn
