#!/bin/bash
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
source /opt/ros/$ROS_DISTRO/setup.bash
