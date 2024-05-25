#!/bin/bash

# Kill all background processes on exit
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

# Redirect all ROS logging output to STDOUT
export RCUTILS_LOGGING_USE_STDOUT=1

./scripts/run-quantum.sh &

./scripts/run-radio.sh &

./scripts/run-thruster.sh &

./scripts/run-imu.sh &
./scripts/run-gps.sh &
./scripts/run-ekf.sh &
# ./scripts/run-navsat_transform_node.sh &
# ./scripts/run-dual-ekf-navsat.sh &

./scripts/run-video.sh &

# Keep this script alive forever
sleep infinity
