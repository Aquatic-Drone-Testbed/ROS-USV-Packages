# order to run the scripts
1. radio-sender | radio-receiver
2. imu          | gps              | ekf |
3. video        | thruster
4. quantum      | radar-on         | ros1&ros2-bridge server
5. navsat_transform_node  | dual-ekf-navsat | ros-bridge   | 

# cmd to check the efk data:
ros2 topic echo /diagnostics
ros2 topic echo /gps/filtered
ros2 topic echo /odometry/filtered 

# how to run-all
1. Connect to PI over SSH
2. Use Docker extension on VSCode
3. Right click on vsc-ros-usv-packages container and "Attach Shell"
4. Run `cd /home/ws`
5. Run `./scripts/run-all.sh`
~~(or run `nohup ./scripts/run-all.sh &` to run in background)~~

# DISCLAIMER  
If you disconnect from the terminal that ran `./scripts/run-all.sh`,
the only way to kill the nodes is by killing the process manually.

You can either a) kill the processes manually or b) restart the entire container

1. Run `ps ax` in container shell and find the PID of `./scripts/run-all.sh`
2. If PID is 1234, run `kill -- -1234`
