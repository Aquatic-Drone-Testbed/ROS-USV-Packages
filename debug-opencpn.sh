#!/bin/bash

cd OpenCPN

sudo apt install devscripts equivs
sudo mk-build-deps -i -r ci/control
sudo apt-get --allow-unauthenticated install -f

cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j `nproc`
sudo cmake --install build
