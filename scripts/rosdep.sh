#!/bin/bash
sudo apt-get update -y && sudo apt-get upgrade -y
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
source /opt/ros/$ROS_DISTRO/setup.bash
