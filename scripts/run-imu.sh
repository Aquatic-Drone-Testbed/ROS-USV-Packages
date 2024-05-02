#!/bin/bash
# To gain permission: chmod +x scripts/run-imu.sh
source /opt/ros/humble/setup.bash
source install/local_setup.bash
echo "Running IMU node..."
sudo chmod a+rw /dev/i2c-*
ros2 launch bno055 bno055.launch.py