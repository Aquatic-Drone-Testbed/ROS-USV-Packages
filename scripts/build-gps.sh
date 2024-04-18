#!/bin/bash
# Script for building and running the GPS driver for ROS2 in non-systemd environments

# Start the gpsd daemon directly (adjust the device path as needed)
sudo gpsd /dev/ttyACM0 -F /var/run/gpsd.sock

# Build the GPS driver
colcon build --symlink-install --packages-select gps_driver

# Check if build is successful
if [ $? -eq 0 ]; then
    echo "GPS build success..."
else
    echo "GPS build failed..."
    exit 1
fi
