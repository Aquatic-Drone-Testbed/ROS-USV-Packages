import roslibpy
import time
import base64
import numpy as np

# ROS2 container's IP address and port
ros2_ip = '172.17.0.1'
ros2_port = 9092

ros2_client = roslibpy.Ros(host=ros2_ip, port=ros2_port)
ros2_client.run()

talker = roslibpy.Topic(ros2_client, '/rosbridge/Polar', 'sensor_msgs/Image')

def callback(message):
    talker.publish(message)
    print(f'sending {message=}')

ros2_listener = roslibpy.Topic(ros2_client, '/Navtech/Polar', 'sensor_msgs/Image')
ros2_listener.subscribe(callback)

try:
    while True:
        pass
except KeyboardInterrupt:
    ros2_client.terminate()
