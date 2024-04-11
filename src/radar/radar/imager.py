import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from radar_interfaces.msg import Spoke

MAX_SPOKE_LENGTH = 128
MAX_INTENSITY = 128
MAX_SPOKE_COUNT = 250

class Imager(Node):
    def __init__(self):
        super().__init__('imager')
        self.publisher_ = self.create_publisher(String, 'radar_control', 10)
        self.subscription = self.create_subscription(
            Spoke,
            'topic_radar_spoke',
            self.listener_callback,
            10)
        
        self.radar_spokes = None


    def listener_callback(self, msg):
        try:
            self.radar_spokes[msg.azimuth, :len(msg.data)] = msg.data
            self.spokes_received += 1
        except:
            pass


    def generate_radar_image(self):
        self.radar_spokes = np.zeros((MAX_SPOKE_COUNT, MAX_SPOKE_LENGTH), np.uint8)
        # self.radar_spokes = np.random.uniform(low=0, high=MAX_INTENSITY, size=(250, MAX_SPOKE_LENGTH)) # random test
        self.spokes_received = 0
        
        msg = String()
        msg.data = 'start_scan'
        self.publisher_.publish(msg)
        
        # block until 250 spokes received
        while self.spokes_received < MAX_SPOKE_COUNT: pass
        
        self.radar_image = cv2.warpPolar(
            src=self.radar_spokes, 
            dsize=(2*MAX_SPOKE_LENGTH, 2*MAX_SPOKE_LENGTH), 
            center=(MAX_SPOKE_LENGTH, MAX_SPOKE_LENGTH), 
            maxRadius=MAX_SPOKE_LENGTH, flags=cv2.WARP_INVERSE_MAP)
        
        msg.data = 'stop_scan'
        self.publisher_.publish(msg)
        
        self.get_logger().info(f'{self.radar_spokes} {self.radar_spokes.shape}')
        self.get_logger().info(f'{self.radar_image} {self.radar_image.shape}')
        cv2.imshow('polar image', self.radar_spokes/MAX_INTENSITY); cv2.waitKey(0)
        cv2.imshow('cartesian image', self.radar_image/MAX_INTENSITY); cv2.waitKey(0)


def main():
    rclpy.init()

    imager_node = Imager()
    imager_node.generate_radar_image()
    imager_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
