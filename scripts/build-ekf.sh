#!/bin/bash
colcon build --packages-select robot_localization

# Check if build is successful
if [ $? -eq 0 ]; then
    echo "robot_localization build success..."
else
    echo "robot_localization build failed..."
    exit 1
fi

#!/bin/bash
# To gain permission: chmod +x scripts/run-ekf.sh
echo "Running EKF node..."
source /opt/ros/humble/setup.bash
source install/setup.bash
# Runs the EKF node using the specified YAML configuration
ros2 launch robot_localization ekf.launch.py