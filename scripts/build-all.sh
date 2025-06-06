#!/bin/bash
# To gain permission: chmod +x scripts/build-all.sh
chmod +x ./scripts/*
./scripts/build-gps.sh
./scripts/build-video.sh
./scripts/rosdep.sh
./scripts/build-thruster.sh
./scripts/build-radio.sh
./scripts/build-radar.sh
./scripts/build-imu.sh
./scripts/build-ros-bridge.sh
./scripts/build-ekf.sh