import rclpy
from rclpy.node import Node
import pigpio
import time

class ThrusterControl(Node):
    def __init__(self):
        super().__init__('thruster_control')
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error('Not connected to Raspberry Pi GPIO. Exiting...')
            rclpy.shutdown()
        
        self.initialize_thrusters()
    
    def delay(self, seconds: float):
        time.sleep(seconds)
    
    def initialize_thrusters(self):
        # Initialize ESC by driving to 1500us for 0.7s
        self.pi.set_servo_pulsewidth(12, 1500)
        self.delay(0.7)
        
        # Example of setting different pulsewidths
        pulsewidths = [1550, 1700, 1800, 1500]
        delays = [5.0, 3.0, 3.0, 0.7]
        
        for pulsewidth, delay_time in zip(pulsewidths, delays):
            self.pi.set_servo_pulsewidth(12, pulsewidth)
            self.delay(delay_time)
        
        self.pi.stop()

def main(args=None):
    rclpy.init(args=args)
    thruster_control = ThrusterControl()
    rclpy.spin(thruster_control)
    thruster_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# # ESC Programming on RPi
# ## Install pigpio for DMA GPIO control
# - Follow instructions listed here to build the pigpio daemon (we will communciate with it over sockets from python calls)
# https://abyz.me.uk/rpi/pigpio/download.html
 
# - Install pigpio python module via pip install pigpio
 
# ## Run daemon
# sudo pigpiod
 
# ## Automate daemon boot
# - I haven't done this yet but might be useful
# https://forums.raspberrypi.com/viewtopic.php?t=319761
 
# # Arduino tutorial
# https://bluerobotics.com/learn/controlling-basic-esc-with-the-arduino-serial-monitor/
 
# #!/usr/bin/env python
 
# import sys
# import time
# import random
 
# import pigpio
 
# pi = pigpio.pi()
 
# if not pi.connected:
#    exit()
 
# def delay(seconds: float):
#    time.sleep(seconds)
 
# # initialze ESC by driving to 1500us for 0.7s
# pi.set_servo_pulsewidth(12, 1500)
# delay(0.7)
 
# pi.set_servo_pulsewidth(12, 1550)
# delay(5.0)
 
# pi.set_servo_pulsewidth(12, 1700)
# delay(3.0)
 
# pi.set_servo_pulsewidth(12, 1800)
# delay(3.0)
 
# pi.set_servo_pulsewidth(12, 1500)
# delay(0.7)
 
# pi.stop()