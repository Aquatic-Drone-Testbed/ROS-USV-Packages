#!/bin/bash
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
