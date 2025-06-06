import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
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

        reentrant_callback_group = ReentrantCallbackGroup()

        # Create a subscriber to the /ldlidar_node/scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/ldlidar_node/scan',
            self.listener_callback,
            10  # QoS profile depth
        )
        
        self.thrust_angle_sub = self.create_subscription(
            String,
            'thrust_angle',
            self.get_angle,
            10  # QoS profile depth
        )

        self.object_detect_publisher = self.create_publisher(
            String, 
            "object_detect", 
            10,
            callback_group=reentrant_callback_group)
        self.lidar_image_publisher = self.create_publisher(
            Image, 
            'lidar_image', 
            10, 
            callback_group=reentrant_callback_group)
        
        self.subscription  # Prevent unused variable warning
        
        # request lidar image every n seconds (n = 0.2 is the fastest, n = 2.5 is less taxing)
        self.lidar_image_timer = self.create_timer(
            0.2, 
            self.generate_lidar_image, 
            reentrant_callback_group)

        self.range_data = ()
        self.spokes = np.zeros((453, 256), np.uint8)
        self.bridge = CvBridge()
        self.t_angle = -1

    def listener_callback(self, msg):
        # Convert the ranges to a tuple to ensure hashability for set storage
        self.range_data = tuple(msg.ranges)
        #angle_inc = msg.angle_increment
        
        # Add to the set
        # self.scan_data_set.add(self.range_data)
        
        vis_range = 2

        for I in range(0, 453):
            #print(self.range_data[I])
            rn = self.range_data[I]
            if (math.isnan(rn) or rn > vis_range):
                pass
            else:
                loc = (int)(rn / vis_range * 255)
                self.spokes[I][loc] = 255

        # Log the size of the set to see how many unique scans are stored
        #self.get_logger().info(f'Unique scans stored: {len(self.scan_data_set)}')
        #print(scan_data)
        tooClose = False
        safe_range = 0.1 #0.04 #EDIT RANGE HERE (in metres)
        for x in range(0, 56):
            if self.range_data[x] < safe_range: 
                #print(x)
                tooClose = True
        for y in range(397,453):
            if self.range_data[y] < safe_range:
                #print(y)
                tooClose = True
        if tooClose:
            msg = String()
            msg.data = "Yes"
            self.object_detect_publisher.publish(msg)
        else:
            msg = String()
            msg.data = "No"
            self.object_detect_publisher.publish(msg)

    def get_angle(self, msg):
        self.t_angle = int(msg.data)

    def generate_lidar_image(self):

        lidar_image = cv2.warpPolar(
            src=self.spokes, 
            dsize=(2*256, 2*256), 
            center=(256, 256), 
            maxRadius=256, flags=cv2.WARP_INVERSE_MAP)
        lidar_image = cv2.rotate(lidar_image, cv2.ROTATE_90_COUNTERCLOCKWISE)

        #Append thrust angle START
        angles = np.zeros((360, 256), np.uint8)

        if self.t_angle != -1:
            for I in range(0, 256):
                angles[self.t_angle][I] = 192

            angle_image = cv2.warpPolar(
            src=angles, 
            dsize=(2*256, 2*256), 
            center=(256, 256), 
            maxRadius=256, flags=cv2.WARP_INVERSE_MAP)

            angle_image = cv2.rotate(angle_image, cv2.ROTATE_90_COUNTERCLOCKWISE)

            lidar_image = angle_image + lidar_image
            


        #Append thrust angle END

        #cv2.imwrite('test/lidar_image.jpg', lidar_image) #print out image to test folder
        #cv2.imwrite(f'test/lidar_stream/lidar_{time.time()}.jpg', lidar_image) #print out stream to test folder

        self.lidar_image_publisher.publish(self.bridge.cv2_to_imgmsg(lidar_image, encoding="passthrough"))

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
