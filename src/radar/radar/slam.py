import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from radar_interfaces.msg import Spoke

MAX_SPOKE_LENGTH = 256
MAX_INTENSITY = 128
MAX_SPOKE_COUNT = 250

class Slam(Node):
    def __init__(self):
        super().__init__('slam')
        self.publisher_ = self.create_publisher(String, 'radar_control', 10)
        self.subscription = self.create_subscription(
            Spoke,
            'topic_radar_spoke',
            self.radar_spoke_callback,
            10)
        
        self.radar_spokes = None # buffer for storing radar spokes

    def radar_spoke_callback(self, spoke):
        self.get_logger().debug(f'Received spoke {spoke.azimuth}')
        if self.radar_spokes is None: return

        self.radar_spokes[spoke.azimuth, :len(spoke.data)] = spoke.data
        self.spokes_received += 1


    def get_random_radar_data(self):
        # initialize buffer to store radar spoke data
        self.radar_spokes = np.random.uniform(low=0, high=MAX_INTENSITY, size=(250, MAX_SPOKE_LENGTH)) # random spokes
        
        radar_data = np.copy(self.radar_spokes/MAX_INTENSITY * 255)
        self.radar_spokes = None # clear buffer
        
        # logging
        # self.get_logger().info(f'{self.radar_spokes=} {self.radar_spokes.shape}')
        cv2.imshow('polar image', radar_data); cv2.waitKey(0)
        
        return radar_data


    def get_radar_data(self):
        # initialize buffer to store radar spoke data
        self.radar_spokes = np.zeros((MAX_SPOKE_COUNT, MAX_SPOKE_LENGTH), np.uint8)
        self.spokes_received = 0
        
        # turn on radar
        msg = String()
        msg.data = 'start_scan'
        self.publisher_.publish(msg)
        
        # block until 250 spokes fill buffer
        while self.spokes_received < MAX_SPOKE_COUNT:
            rclpy.spin_once(self)
        
        # turn off radar
        msg.data = 'stop_scan'
        self.publisher_.publish(msg)
        
        radar_data = np.copy(self.radar_spokes/MAX_INTENSITY * 255).astype(np.uint8)
        
        # logging
        # self.get_logger().info(f'{self.radar_spokes=} {self.radar_spokes.shape}')
        cv2.imshow('polar image', radar_data); cv2.waitKey(0)
        
        self.radar_spokes = None # clear buffer
        
        return radar_data


    def generate_map(self, r, k, p, K, w, gamma):
        """Coastline Extraction and Parameterization algorithm from
        https://ieeexplore.ieee.org/document/8600301

        Args:
            r (_type_): raw radar data
            k (_type_): spline curve degree
            p (_type_): knot spacing
            K (_type_): coordinate transformation matrix
            w (_type_): minimum polygon area threshold
            gamma (_type_): angular resolution to discretize the polar coordinate 
        """
        # I = self.generate_radar_image(r, K)
        I = cv2.imread('/home/ws/test.png', cv2.IMREAD_GRAYSCALE)
        D = self.detect_contour(self.filter_image(I))
        P = self.extract_coastline(D, w, gamma, K)
        
        # psuedocode
        # P={P1,…,Pn} , F⟵∅

        # for i ← 1 to n do

        # Pi={p0,…,pm}

        # ł=∑j=1m∥pj−pj−1∥

        # B⟵ CoxdeBoorRecursion(Pi,ρ,l, k)

        # C⟵ SplineCurveFitting(Pi,B)

        # F⟵F⋃C

        # end for
        
        ############## end of pseudocode ##############


    def generate_radar_image(self, radar_data, K):
        """convert 2d polar image of radar data to 2d cartesian image

        Args:
            radar_data (_type_): 2D array of dimension (MAX_SPOKE_COUNT, MAX_SPOKE_LENGTH)
                                 with each row representing a single radar spoke

        Returns:
            radar_image (_type_): grayscale image of radar scan in cartesian coordinates
        """
        radar_image = cv2.warpPolar(
            src=radar_data, 
            dsize=(4*MAX_SPOKE_LENGTH, 4*MAX_SPOKE_LENGTH), 
            center=(2*MAX_SPOKE_LENGTH, 2*MAX_SPOKE_LENGTH), 
            maxRadius=2*MAX_SPOKE_LENGTH, flags=cv2.WARP_INVERSE_MAP)
        radar_image = cv2.rotate(radar_image, cv2.ROTATE_90_CLOCKWISE)
        
        # self.get_logger().info(f'{radar_image=} {radar_image.shape}')
        cv2.imshow('cartesian image', radar_image); cv2.waitKey(0)
        
        return radar_image


    def filter_image(self, radar_image, binary_threshold=128):
        """apply morphological and bilateral filters for denoising
        and convert grayscale radar_image to binary image according to 
        predetermined threshold intensity value
        
        Args:
            radar_image (_type_): grayscale image of radar scan in cartesian coordinates

        Returns:
            _type_: _description_
        """
        cv2.imshow('radar image', radar_image); cv2.waitKey(0)
        
        # [TODO]: apply morphological and bilateral filters
        opening = cv2.morphologyEx(radar_image, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        cv2.imshow('opening image', opening); cv2.waitKey(0)
        
        bilateral = cv2.bilateralFilter(opening, 9, 400, 400)
        cv2.imshow('bilateral image', bilateral); cv2.waitKey(0)
        
        # [TODO]: adjust threshold intensity value. range:[0,255]
        _, binary_image = cv2.threshold(bilateral, binary_threshold, 255, cv2.THRESH_BINARY)
        cv2.imshow('filtered image', binary_image); cv2.waitKey(0)
        
        return binary_image


    def detect_contour(self, binary_image, area_threshold=50):
        """extract the contours from the binary image using polygon extraction

        Args:
            binary_image (_type_): _description_

        Returns:
            _type_: _description_
        """
        # [TODO]: extract the contours from the binary image using polygon extraction
        
        contours, hierarchy = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        landmasses = [cnt for cnt in contours if cv2.contourArea(cnt) > area_threshold]
        
        img = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2RGB) 
        cv2.drawContours(img, landmasses, -1, (0,255,0), 3)
        cv2.imshow('contoured image', img); cv2.waitKey(0)
        
        return landmasses


    def extract_coastline(self, D, w, gamma, K):
        pass


def main():
    rclpy.init()

    slam_node = Slam()
    # radar_data = slam_node.get_radar_data()
    radar_data = slam_node.get_random_radar_data()
    slam_node.generate_map(r=radar_data, k=None, p=None, K=None, w=None, gamma=None)
    slam_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
