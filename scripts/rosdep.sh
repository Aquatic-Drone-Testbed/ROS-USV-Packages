#!/bin/bash
#Script for setting up ros dependencies
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
source /opt/ros/$ROS_DISTRO/setup.bash