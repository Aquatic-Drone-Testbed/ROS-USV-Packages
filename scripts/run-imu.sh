#!/bin/bash
# To gain permission: chmod +x scripts/run-imu.sh
source /opt/ros/humble/setup.bash
source install/local_setup.bash
echo "Running IMU node..."
sudo chmod a+rw /dev/i2c-*
ros2 run bno055 bno055 --ros-args --params-file ./src/bno055/bno055/params/bno055_params_i2c.yaml
