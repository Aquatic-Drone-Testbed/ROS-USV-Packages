#!/bin/bash
# To gain permission: chmod +x scripts/run-video.sh
source install/setup.bash
echo "Running Video node..."
ros2 run video_stream_py video_publisher --ros-args --log-level info


# udp.port == 39000 or  udp.port == 39001 or udp.port == 39002 or udp.port == 39003 or udp.port == 39004 or udp.port == 39005