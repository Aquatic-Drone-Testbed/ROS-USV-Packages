#!/bin/bash
# To gain permission: chmod +x scripts/build-all.sh

./build-gps.sh
./build-video.sh
./rosdep.sh
./build-radio.sh