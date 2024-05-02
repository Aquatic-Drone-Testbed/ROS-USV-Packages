#!/bin/bash
colcon build --packages-select robot_localization

# Check if build is successful
if [ $? -eq 0 ]; then
    echo "robot_localization build success..."
else
    echo "robot_localization build failed..."
    exit 1
fi
