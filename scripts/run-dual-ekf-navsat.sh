
#!/bin/bash
# To gain permission: chmod +x scripts/run-ekf.sh
echo "Running Dual EKF node..."
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch robot_localization dual_ekf_navsat_example.launch.py