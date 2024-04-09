# ROS-USV-Packages

This repository can be used to create a vscode-docker ROS2 Humble dev environment on any host OS.  
Environment setup is based on [ROS2 Community Guide](https://docs.ros.org/en/humble/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html?highlight=vscode)

## Installation

1. Install Docker
2. Install Remote Development extension in vscode
3. Clone and open this repository in vscode
4. Open Command Palette and run "Dev Containers: Rebuild and Reopen in Container"
5. Wait for container to install

## Building ros_tutorial package

1. run `git submodule update --init --recursive` to clone all the submodules
2. run `colcon build`

## Camera Things

1. Run build-video.sh
2. Run rosdep.sh
3. Build video/radio packages
4. If it says no permission, need to add the user to the video group
5. sudo usermod -a -G video $USER
6. Power cycle, should be fine. To test, do ls /dev/video\* and you should see some directories appear that don't appear when camera isn't plugged in
7. ros2 run video_stream_py video_publisher
8. ros2 run radio sender_udp
9. On control station, run image_receive.py
