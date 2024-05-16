#!/bin/bash
# Script for building video streaming package

./scripts/rosdep.sh
pip install cv_bridge
colcon build --symlink-install --packages-select radar

# Check if build is successful
if [ $? -eq 0 ]; then
    echo "Radar build success..."
else
    echo "Radar build failed..."
    exit 1
fi