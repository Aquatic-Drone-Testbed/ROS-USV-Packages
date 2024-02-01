#!/bin/bash
# Script for building radar-pi with debug flag

cd /home/ws/radar_pi

sudo apt install devscripts equivs gdb
sudo mk-build-deps -i -r ci/control
sudo apt-get --allow-unauthenticated install -f

cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j `nproc`
