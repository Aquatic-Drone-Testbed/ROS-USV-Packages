import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from rclpy.callback_groups import ReentrantCallbackGroup
import time
import math
import numpy as np
import cv2

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        
        # Initialize an empty set to store unique scan data
        self.scan_data_set = set()

        # Create a subscriber to the /ldlidar_node/scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/ldlidar_node/scan',
            self.listener_callback,
            10  # QoS profile depth
        )
        self.subscription  # Prevent unused variable warning
        
        reentrant_callback_group = ReentrantCallbackGroup()
        
        # request lidar image every n seconds (n = 0.2 is the fastest, n = 2.5 is less taxing)
        self.lidar_image_timer = self.create_timer(
            2.5, 
            self.generate_lidar_image, 
            reentrant_callback_group)

        self.range_data = ()
        self.spokes = np.zeros((453, 256), np.uint8)

    def listener_callback(self, msg):
        # Convert the ranges to a tuple to ensure hashability for set storage
        self.range_data = tuple(msg.ranges)
        #angle_inc = msg.angle_increment
        
        # Add to the set
        # self.scan_data_set.add(self.range_data)
        
        for I in range(0, 453):
            #print(self.range_data[I])
            rn = self.range_data[I]
            if (math.isnan(rn) or rn > 3):
                pass
            else:
                loc = (int)(rn / 3 * 255)
                self.spokes[I][loc] = 255

        # Log the size of the set to see how many unique scans are stored
        #self.get_logger().info(f'Unique scans stored: {len(self.scan_data_set)}')
        #print(scan_data)
        tooClose = False
        for x in range(0, 56):
            if self.range_data[x] < 0.04:
                #print(x)
                tooClose = True
        for y in range(397,453):
            if self.range_data[y] < 0.04:
                #print(y)
                tooClose = True
        if tooClose:
            print("Too Close")
        else:
            print(" ")

    def generate_lidar_image(self):

        lidar_image = cv2.warpPolar(
            src=self.spokes, 
            dsize=(2*256, 2*256), 
            center=(256, 256), 
            maxRadius=256, flags=cv2.WARP_INVERSE_MAP)
        lidar_image = cv2.rotate(lidar_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        
        cv2.imwrite('test/lidar_image.jpg', lidar_image)
        #cv2.imwrite(f'test/lidar_stream/lidar_{time.time()}.jpg', lidar_image)
        self.spokes = np.zeros((453, 256), np.uint8)

def main(args=None):
    rclpy.init(args=args)

    lidar_sub = LidarSubscriber()

    try:
        rclpy.spin(lidar_sub)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        lidar_sub.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
