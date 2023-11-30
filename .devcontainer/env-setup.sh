#!/bin/bash
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

# Uncomment below lines to install ROS turtle tutorial after creating dev container
# sudo apt update
# sudo apt install -y ros-humble-turtlesim ~nros-humble-rqt*
