#!/bin/bash
#biulds and runs the IMU 
# Navigate to your ROS 2 workspace
cd /home/ws

# Build the workspace
echo "Building ROS 2 workspace..."
colcon build --packages-select bno055 --symlink-install

# Source the ROS 2 setup file
echo "Sourcing setup files..."
source /opt/ros/humble/setup.bash
source install/local_setup.bash

# Run your ROS 2 node
echo "Running the IMU node..."
sudo chmod a+rw /dev/i2c-*
ros2 run bno055 bno055 --ros-args --params-file ./src/bno055/bno055/params/bno055_params_i2c.yaml
