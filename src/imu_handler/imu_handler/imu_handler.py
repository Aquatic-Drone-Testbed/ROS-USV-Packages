import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from std_msgs.msg import String

from rclpy.callback_groups import ReentrantCallbackGroup
#import time
import math
#import numpy as np
#import cv2

class ImuHandler(Node):
    def __init__(self):
        super().__init__('nav_module')

        reentrant_callback_group = ReentrantCallbackGroup()
        
        self.subscription_imu = self.create_subscription(
            Imu, 
            '/bno055/imu', 
            self.process_imu,  #receives messages/ processes controller input``
            10)

        self.subscription_mag  = self.create_subscription(
            MagneticField, 
            '/bno055/mag', 
            self.receive_mag,  #receives messages/ processes controller input``
            10)
        
        self.imu_publisher = self.create_publisher(
            String,
            "imu_handler_data",
            10,
            callback_group=reentrant_callback_group
        )

        self.compass = 0

    def process_imu(self, msg):
        ort = msg.orientation
        ang_v = msg.angular_velocity
        lin_a = msg.linear_acceleration
        out_msg = String()
        out_msg.data = f'ORIENT<{ort}>\tANG_VEL<{ang_v}>\tLIN_ACC<{lin_a}>\tCOMP<{self.compass}>\n'
        self.imu_publisher.publish(out_msg)
        #print(angv)
        #v_x = angv.x
        #v_y = angv.y
        #v_z = ang_v.z
        #print(f'{v_z:.2f}')
            
    def receive_mag(self, msg):
        mag = msg.magnetic_field
        heading = math.atan2(mag.y,mag.x)

        if heading >= 0:
            heading = heading * 180/math.pi
        else: 
            heading = (2*math.pi + heading) * 180/math.pi

        self.compass = int(heading)
        #print(int(heading))

def main(args=None):
    rclpy.init(args=args)

    imu_handler = ImuHandler()

    try:
        rclpy.spin(imu_handler)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        imu_handler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
