#!/bin/bash
# Script for building and installing openCPN with debug flag

cd /home/ws/OpenCPN

sudo apt install devscripts equivs gdb
sudo mk-build-deps -i -r ci/control
sudo apt-get --allow-unauthenticated install -f

cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j `nproc`
sudo cmake --install build