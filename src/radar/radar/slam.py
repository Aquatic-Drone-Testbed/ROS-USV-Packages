import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from radar_interfaces.msg import Spoke
from sensor_msgs.msg import Image

MAX_SPOKE_LENGTH = 256
MAX_INTENSITY = 128
MAX_SPOKE_COUNT = 250

class Slam(Node):
    def __init__(self):
        super().__init__('slam')
        self.image_publisher_ = self.create_publisher(Image, 'radar_image', 10)
        self.publisher_ = self.create_publisher(String, 'radar_control', 10)
        self.subscription = self.create_subscription(
            Spoke,
            'topic_radar_spoke',
            self.radar_spoke_callback,
            10)
        
        self.radar_spokes = None # buffer for storing radar spokes
        self.bridge = CvBridge()


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
        # cv2.imshow('polar image', radar_data); cv2.waitKey(0)
        
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
        # cv2.imshow('polar image', radar_data); cv2.waitKey(0)
        
        self.radar_spokes = None # clear buffer
        
        return radar_data


    def generate_map(self, r, k, p, K, area_threshold, gamma):
        """Coastline Extraction and Parameterization algorithm from
        https://ieeexplore.ieee.org/document/8600301

        Args:
            r (_type_): raw radar data
            k (_type_): spline curve degree
            p (_type_): knot spacing
            K (_type_): coordinate transformation matrix
            area_threshold (_type_): minimum polygon area threshold
            gamma (_type_): angular resolution to discretize the polar coordinate 
        """
        # I = self.generate_radar_image(r, K)
        I = cv2.imread('/home/ws/test.png', cv2.IMREAD_GRAYSCALE)
        I = cv2.resize(I, None, fx=0.5, fy=0.5)
        D = self.detect_contour(self.filter_image(I,binary_threshold=128))
        P = self.extract_coastline(D, area_threshold=10, angular_resolution=None, K=None)
        
        final = cv2.cvtColor(I,cv2.COLOR_GRAY2RGB)
        print(final.shape)
        print(P.shape)
        # cv2.imshow('final', final); cv2.waitKey(0)
        
        self.image_publisher_.publish(self.bridge.cv2_to_imgmsg(P, encoding="passthrough"))
        self.get_logger().info(f'Published coastline image')


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
        """convert 2d polar image of radar data to 2d cartesian imagecv2.im

        Args:
            radar_data (_type_): 2D array of dimension (MAX_SPOKE_COUNT, MAX_SPOKE_LENGTH)
                                 with each row representing a single radar spoke

        Returns:
            radar_image (_type_): grayscale image of radar scan in cartesian coordinates
        """
        radar_image = cv2.warpPolar(
            src=radar_data, 
            dsize=(2*MAX_SPOKE_LENGTH, 2*MAX_SPOKE_LENGTH), 
            center=(MAX_SPOKE_LENGTH, MAX_SPOKE_LENGTH), 
            maxRadius=MAX_SPOKE_LENGTH, flags=cv2.WARP_INVERSE_MAP)
        radar_image = cv2.rotate(radar_image, cv2.ROTATE_90_CLOCKWISE)
        
        # self.get_logger().info(f'{radar_image=} {radar_image.shape}')
        # cv2.imshow('raw image', radar_image); cv2.waitKey(0)
        
        return radar_image


    def filter_image(self, radar_image, binary_threshold):
        """apply morphological and bilateral filters for denoising
        and convert grayscale radar_image to binary image according to 
        predetermined threshold intensity value
        
        Args:
            radar_image (_type_): grayscale image of radar scan in cartesian coordinates

        Returns:
            _type_: _description_
        """
        
        # [TODO]: apply morphological and bilateral filters
        erosion = cv2.erode(radar_image, np.ones((3, 3), np.uint8), iterations=1)
        # cv2.imshow('erosion', erosion); cv2.waitKey(0)
        dilation = cv2.dilate(erosion, np.ones((3, 3), np.uint8), iterations=1)
        # cv2.imshow('dilation', dilation); cv2.waitKey(0)
        
        bilateral = cv2.bilateralFilter(dilation, 9, 100, 100)
        # cv2.imshow('bilateral', bilateral); cv2.waitKey(0)
        
        # [TODO]: adjust threshold intensity value. range:[0,255]
        _, binary_image = cv2.threshold(bilateral, binary_threshold, 255, cv2.THRESH_BINARY)
        # cv2.imshow('threshold', binary_image); cv2.waitKey(0)
        
        return binary_image


    def detect_contour(self, binary_image):
        """extract the contours from the binary image using polygon extraction

        Args:
            binary_image (_type_): _description_

        Returns:
            _type_: _description_
        """
        # [TODO]: extract the contours from the binary image using polygon extraction
        
        contours, hierarchy = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return contours


    def extract_coastline(self, contours, area_threshold, angular_resolution=None, K=None):
        landmasses = [cnt for cnt in contours if cv2.contourArea(cnt) > area_threshold]
        
        img = np.zeros((2*MAX_SPOKE_LENGTH, 2*MAX_SPOKE_LENGTH, 3), dtype=np.uint8)
        cv2.drawContours(img, landmasses, -1, (255,255,255), -1)
        # cv2.imshow('contour', img); cv2.waitKey(0)
        
        height, width = img.shape[:2]
        polar = cv2.warpPolar(src=img, dsize=(MAX_SPOKE_LENGTH, 0), center=(width/2, height/2), maxRadius=width/2, flags=cv2.WARP_POLAR_LINEAR)
        polar = cv2.cvtColor(polar, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('polar', polar); cv2.waitKey(0)
        
        for spoke in polar:
            spoke[np.argmax(spoke > 0)+1:] = 0
        
        # cv2.imshow('coastline polar', polar); cv2.waitKey(0)
        
        coastline = cv2.warpPolar(
            src=polar, 
            dsize=(2*MAX_SPOKE_LENGTH, 2*MAX_SPOKE_LENGTH), 
            center=(MAX_SPOKE_LENGTH, MAX_SPOKE_LENGTH), 
            maxRadius=MAX_SPOKE_LENGTH, flags=cv2.WARP_INVERSE_MAP)
        
        # cv2.imshow('coastline', coastline); cv2.waitKey(0)
        
        return coastline


def main():
    rclpy.init()

    slam_node = Slam()
    # radar_data = slam_node.get_radar_data()
    radar_data = slam_node.get_random_radar_data()
    slam_node.generate_map(r=radar_data, k=None, p=None, K=None, area_threshold=50, gamma=None)
    slam_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
