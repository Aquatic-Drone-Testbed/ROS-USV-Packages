#!/bin/bash
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
sudo rosdep update
sudo rosdep install --from-paths src --ignore-src -y
