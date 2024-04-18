#!/bin/bash

echo "Running radio_sender node..."
source install/setup.bash
ros2 run radio sender_udp --ros-args -p target_ip:='10.223.75.164'
