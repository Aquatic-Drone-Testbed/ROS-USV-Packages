#!/bin/bash
# Script for building video streaming package

source install/setup.bash
sudo apt install libopencv-dev -y
sudo apt install ros-humble-cv-bridge -y
sudo apt-get update && sudo apt-get upgrade -y