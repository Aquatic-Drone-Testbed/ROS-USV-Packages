#!/bin/bash
# To gain permission: chmod +x scripts/build-all.sh
chmod +x ./scripts/build-gps.sh
chmod +x ./scripts/build-video.sh
chmod +x ./scripts/rosdep.sh
chmod +x ./scripts/build-thruster.sh
chmod +x ./scripts/build-radio.sh
chmod +x ./scripts/run-gps.sh
chmod +x ./scripts/run-radio-receiver.sh
chmod +x ./scripts/run-radio-sender.sh
chmod +x ./scripts/run-thruster.sh
chmod +x ./scripts/run-video.sh
./scripts/build-gps.sh
./scripts/build-video.sh
./scripts/rosdep.sh
./scripts/build-thruster.sh
./scripts/build-radio.sh