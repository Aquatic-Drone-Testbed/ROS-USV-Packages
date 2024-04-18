#!/bin/bash

echo "Running radio_receiver node..."
source install/setup.bash
ros2 run radio receiver_udp --ros-args -p host:='0.0.0.0'
