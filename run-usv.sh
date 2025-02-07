#!/bin/bash

# Kill all background processes on exit
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

# Redirect all ROS logging output to STDOUT
export RCUTILS_LOGGING_USE_STDOUT=1

# ./scripts/run-quantum.sh &

./scripts/run-radio.sh &

# ./scripts/run-thruster.sh &

# ./scripts/run-imu.sh &
# ./scripts/run-gps.sh &
# ./scripts/run-ekf.sh &
# ./scripts/run-navsat_transform_node.sh &
# ./scripts/run-dual-ekf-navsat.sh &

# ./scripts/run-video.sh &

./scripts/run-lidar.sh &
./scripts/run-lidar-sub.sh &

# Keep this script alive forever

# ros2 topic pub /radar_control std_msgs/String '{data: start_scan}' -1
# ros2 topic pub /radar_control std_msgs/String '{data: stop_scan}' -1

# sudo route add -net 224.0.0.0 netmask 224.0.0.0 enxa0cec8b67b9f




sleep infinity
