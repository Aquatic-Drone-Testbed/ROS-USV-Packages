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

## Instruction to Build at frist time

1. Run the build script:
    ```bash
    ./scripts/build-all.sh
    ```

2. (Optional) If you encounter a permission error, add your user to the video group:
    ```bash
    sudo usermod -a -G video $USER
    ```

3. (Optional) Power cycle your system. To verify the camera is recognized post-power cycle, run the following command to see directories that should appear when the camera is connected:
    ```bash
    ls /dev/video*
    ```

# Instruction to Run

Run the following commands in separate terminals:
- Start the GPS service:
    ```bash
    ./run-gps.sh
    ```
- Start the video service:
    ```bash
    ./run-video.sh
    ```
- Start the thruster service:
    ```bash
    ./run-thruster.sh
    ```
- (TODO) Start the radar service:
    ```bash
    ./run-radar.sh
    ```
- Start the radio sender service:
    ```bash
    ./run-radio-sender.sh
    ```
- Start the radio receiver service:
    ```bash
    ./run-radio-receiver.sh
    ```



