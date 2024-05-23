#!/bin/bash
source install/setup.bash
ros2 topic pub /radar_control std_msgs/String '{data: zoom_out}' -1