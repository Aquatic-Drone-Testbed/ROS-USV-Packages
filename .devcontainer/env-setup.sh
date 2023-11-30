#!/bin/bash
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
rosdep update
rosdep install -i --from-path src --rosdistro iron -y

# Uncomment below lines to install ROS turtle tutorial after creating dev container
# sudo apt update
# sudo apt install -y ros-iron-turtlesim ~nros-iron-rqt*
