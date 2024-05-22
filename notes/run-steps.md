# order to run the scripts
1. radio-sender | radio-receiver
2. imu          | gps              | quantum | video | thruster
3. efk          | radar-on         | ros1&ros2-bridge server
4. run-navsat_transform_node  | run-dual-ekf-navsat | ros-bridge   | 

# cmd to check the efk data:
ros2 topic echo /diagnostics
ros2 topic echo /gps/filtered
ros2 topic echo /odometry/filtered 