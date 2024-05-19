import roslibpy
import time
import base64
import numpy as np

# ros1 container's IP address and port
ros1_ip = 'localhost'
ros1_port = 9090

ros1_client = roslibpy.Ros(host=ros1_ip, port=ros1_port)
ros1_client.run()

talker = roslibpy.Topic(ros1_client, '/Navtech/Polar', 'sensor_msgs/Image')

def callback(message):
    talker.publish(message)
    print(f'received image over socket')

ros1_listener = roslibpy.Topic(ros1_client, '/rosbridge/Polar', 'sensor_msgs/Image')
ros1_listener.subscribe(callback)

try:
    while True:
        pass
except KeyboardInterrupt:
    ros1_client.terminate()
