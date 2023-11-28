#!/bin/bash
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
sudo rosdep update
sudo rosdep install --from-paths src --ignore-src -y

# Uncomment below lines to install ROS turtle tutorial after creating dev container
# sudo apt update
# sudo apt install -y ros-iron-turtlesim ~nros-iron-rqt*
