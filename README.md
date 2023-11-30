# ROS-demo
This repository can be used to create a vscode-docker ROS2 Humble dev environment on any host OS.  
Environment setup is based on [ROS2 Community Guide](https://docs.ros.org/en/humble/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html?highlight=vscode)  

## Installation
1. Install Docker
2. Install Remote Development extension in vscode
2. Clone and open this repository in vscode
3. Open Command Palette and run "Dev Containers: Rebuild and Reopen in Container"
4. Wait for container to install

## Building ros_tutorial package
1. run `git submodule update --init --recursive` to pull the ros_tutorials repository
2. run `colcon build`
