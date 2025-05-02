#!/bin/bash

source ~/.bashrc
echo "Running udev setup for Lidar..."
# sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
echo "udev setup for for Lidar complete!"
echo "Running LD Lidar node..."
ros2 launch ldlidar_node ldlidar_with_mgr.launch.py
