import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
#import time
import math
#import numpy as np
#import cv2
import time

class NavigationModule(Node):
    def __init__(self):
        super().__init__('nav_module')

        reentrant_callback_group = ReentrantCallbackGroup()
        command_mutex_callback_group = MutuallyExclusiveCallbackGroup()
        
        # timers
        # run every 2 seconds to clear recent detect flag
        self.standby_timer = self.create_timer(
            2, 
            self.detect_timer_callback,
            reentrant_callback_group)

        self.subscription = self.create_subscription(
            String, 
            'thruster_control', 
            self.update_direction,  #receives messages/ processes controller input``
            10)
        
        self.object_detect_subscriber = self.create_subscription(
            String, 
            "object_detect", 
            self.block_direction,
            10)
        
        self.navmod_control_subscription = self.create_subscription(
            String,
            'navmod_control',
            self.navmod_control_callback,
            10,
            callback_group=command_mutex_callback_group)
        
        self.thrust_publisher = self.create_publisher(
            String,
            "thruster_input",
            10,
            callback_group=reentrant_callback_group
        )

        self.thrust_angle_publisher = self.create_publisher(
            String,
            "thrust_angle",
            10,
            callback_group=reentrant_callback_group
        )
        
        self.subscription  # Prevent unused variable warning
        self.base_thrust = 0
        self.delta_thrust = 0
        self.detect_flag = 0
        self.recent_flag = 0
        self.navmod_enabled = True

    def block_direction(self, msg):
        if msg.data == "Yes":
            self.detect_flag = 1
        else:
            self.detect_flag = 0
            
    def update_direction(self, msg):
        if msg.data == "RADIO_TIMEOUT":
            self.get_logger().warn('Radio timed out... Resetting thrusters to 0')
            self.base_thrust = 0
            self.delta_thrust = 0
            mes = String()
            mes.data = str(self.base_thrust) + "," + str(self.delta_thrust)
            #print(mes.data)
            self.thrust_publisher.publish(mes)
            mesv = String()
            mesv.data = str(-1)
            self.thrust_angle_publisher.publish(mesv)
            return
        
        #Adjust thruster values
        direction, value_str = msg.data.split(',')
        
        value = float(value_str) # [-1, 1]
        
        #print(f"{direction}-{value}")

        # #Taper control

        # nvalue = 0
        # if abs(value) <= 0.12 and abs(value) > 0.001:
        #     if value < 0:
        #         nvalue = -0.12
        #     else:
        #         nvalue = 0.12

        # if abs(value) > 0.12:
        #     if value < 0:
        #         nvalue = -(abs(value)*0.45 + 0.07)
        #     else:
        #         nvalue = value*0.3+0.08

        # #Turbo
        # if abs(value) == 1.0:
        #     if value < 0:
        #         nvalue = -1.0
        #     else:
        #         nvalue = 1.0

        # value = nvalue

        # # Speed governor
        # if abs(value) > 0.5:
        #    if value < 0:
        #        value = -0.5
        #    else:
        #        value = 0.5

        #print(value)

        if direction == "ABS_Y":
            #NEW CODE
            if self.detect_flag == 1 and value > 0 and self.navmod_enabled:
                self.base_thrust = 0
                self.recent_flag = 1
                #if self.delta_thrust == 0:
                self.delta_thrust = 0.5
            else:
                self.base_thrust = value
        elif direction == "ABS_X":
            self.delta_thrust = value
        
        if self.detect_flag == 0 and self.recent_flag == 1:
            self.base_thrust = 0.5
            self.delta_thrust = 0

        ### Angle Display Code START

        if self.base_thrust == 0 and self.delta_thrust == 0:
            dir_angle = -1
        else:
            dir_angle = math.atan2(self.delta_thrust,self.base_thrust)

            if dir_angle >= 0:
                dir_angle = dir_angle * 180/math.pi
            else: 
                dir_angle = (2*math.pi + dir_angle) * 180/math.pi

        #print(int(dir_angle))

        mesv = String()
        mesv.data = str(int(dir_angle))
        self.thrust_angle_publisher.publish(mesv)

        ### Angle Display Code END

        mes = String()
        mes.data = str(self.base_thrust) + "," + str(self.delta_thrust)
        #print(mes.data)
        self.thrust_publisher.publish(mes)
        if self.detect_flag == 0 and self.recent_flag == 1:
            #time.sleep(2)
            self.recent_flag = 0

    def navmod_control_callback(self, msg):
        match msg.data:
            case 'toggle_navmod':
                self.navmod_enabled = not self.navmod_enabled
                print(f'navmod_enabled is: {self.navmod_enabled}')
            case _:
                self.get_logger().error(f'unknown navmod control: {msg.data}')

    def detect_timer_callback(self):
        if self.detect_flag == 0 and self.recent_flag == 1:
            self.recent_flag = 0

def main(args=None):
    rclpy.init(args=args)

    nav_module = NavigationModule()

    try:
        rclpy.spin(nav_module)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        nav_module.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
