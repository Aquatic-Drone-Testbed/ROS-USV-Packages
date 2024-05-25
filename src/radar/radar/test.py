import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import radar.filter as RadarFilter

class Test(Node):
    def __init__(self):
        super().__init__('test_node')
        self.subscription = self.create_subscription(
            Image,
            'USV/Polar', #'NavTech/Polar'
            self.write_image,
            10)
        
        self.bridge = CvBridge()

    def write_image(self, img):
        self.get_logger().info(f'received new image')
        polar_image = self.bridge.imgmsg_to_cv2(img)
        cv2.imwrite('test/polar_image.jpg', polar_image)
        
        RadarFilter.generate_map(r=polar_image, k=None, p=None, K=None, area_threshold=50, gamma=None)


def main():
    rclpy.init()

    test_node = Test()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
