#!/bin/bash
# Script for building thruster control package
source install/setup.bash
sudo apt install wget unzip -y
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install
sudo apt-get install python-setuptools python3-setuptools python3-pigpio python3-pip ros-humble-joy -y
pip install pigpio setuptools==58.2.0 
sudo pigpiod
cd ..
rm master.zip
sudo apt-get update && sudo apt-get upgrade -y