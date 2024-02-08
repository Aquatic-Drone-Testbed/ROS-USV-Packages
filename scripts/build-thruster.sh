#!/bin/bash
# Script for building thruster control package
sudo apt install wget unzip -y
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install
sudo apt-get install python-setuptools python3-setuptools python3-pigpio -y
pip install pigpio
sudo pigpiod
cd ..
sudo apt-get update && sudo apt-get upgrade -y
source scripts/rosdep.sh