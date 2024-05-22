#!/bin/bash
# To gain permission: chmod +x scripts/run-radio-sender.sh
echo "Running ros brige server..."
source install/setup.bash

# using port 9092 as the server's port
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9092