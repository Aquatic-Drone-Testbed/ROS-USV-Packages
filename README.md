# ROS-USV-Packages

This repository can be used to create a vscode-docker ROS2 Humble dev environment on any host OS.  
Environment setup is based on [ROS2 Community Guide](https://docs.ros.org/en/humble/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html?highlight=vscode)

## Pi5 GPIO PWM Setup

Due to hardware changes on the Pi5, you may need to update the Pi5 with custom firmware to enable hardware PWM on the GPIO pins. The instructions to do so are here: [https://gist.github.com/Gadgetoid/b92ad3db06ff8c264eef2abf0e09d569] (Note: The "safety rail" script mentioned was not used in any of our implementations and is untested with our setup)

Once this is done, you can use the python package ```rpi-hardware-pwm```. There is also a permissions setting file ```pwm-permissions.sh``` and python script ```pwm-init.py``` that need to be executed the first time on each boot for the GPIO pins to work. Generally, you will not need to mess with this as it's handled by other launch scripts but you may need to include these in any custom launch scripts you write.

You may also need to add a ```gpio``` group in the docker itself and add the user to this group. Additionally, outside the docker on the host, you may need to add the ```docker``` user to the ```sys``` group. Only do this if you are still having issues.

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
- Start the radar service:
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



