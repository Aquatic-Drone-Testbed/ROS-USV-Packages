# In ros1:
roslaunch rosbridge_server rosbridge_websocket.launch

# In ros2:
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9092

source /opt/ros/noetic/setup.bash
source /opt/ros/humble/setup.bash