#!/bin/bash
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
