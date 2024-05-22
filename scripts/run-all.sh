#!/bin/bash

export RCUTILS_LOGGING_USE_STDOUT=1

# ./scripts/run-quantum.sh &
# ./scripts/run-thruster.sh &
# ./scripts/run-radio-receiver.sh &

# ./scripts/run-imu.sh &
# ./scripts/run-gps.sh &
# ./scripts/run-ekf.sh &

# ./scripts/run-radio-sender.sh &
# ./scripts/run-video.sh &

trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM
