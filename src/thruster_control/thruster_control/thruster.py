import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Joy
import pigpio
import time

class ThrusterControl(Node):
    def __init__(self):
        super().__init__('thruster_control')
        self.pi = pigpio.pi() # connect to pi
        if not self.pi.connected:
            self.get_logger().error('Not connected to Raspberry Pi GPIO. Exiting...')
            rclpy.shutdown()

        self.joy_subscription = self.create_subscription( # subscribes to the joystick input from joy node
            Joy,
            '/joy',
            self.joy_callback,
            10,
            callback_group=ReentrantCallbackGroup())
        
        self.esc1_pulsewidth = 1500 # Initial startup for ESC 1
        self.esc2_pulsewidth = 1500 # Initial startup for ESC 2
        self.delay(0.7)
        self.esc1_axis_value = 0  # Gets and stores latest joystick position
        self.esc2_axis_value = 0
        self.pi.set_servo_pulsewidth(12, self.esc1_pulsewidth)  # gpio pin is 12
        self.pi.set_servo_pulsewidth(13, self.esc2_pulsewidth)  # gpio pin is 12
        
        self.timer = self.create_timer(0.1, self.timer_callback)  # Adjust PWM vaue at most every 0.1 seconds
        
        self.get_logger().info('Thruster control initialized.')

        print(f"{'Timestamp':<20} | {'ESC 1 Pulsewidth':<17} | {'ESC 2 Pulsewidth'}")
        print("-" * 60)  # Print a separator line

    def joy_callback(self, msg):
        # Sets axis to be vertical joystick axis only
        self.esc1_axis_value = msg.axes[1]
        self.esc2_axis_value = msg.axes[4]

    def timer_callback(self):
        # Use the stored axis value to adjust PWM value
        adjustment = self.esc1_axis_value * 10  # Smaller scale factor for smoother adjustment
        new_pulsewidth = max(1100, min(1900, self.esc1_pulsewidth + adjustment))
        if new_pulsewidth != self.esc1_pulsewidth: #updates PWM value if joystick moved
            self.pi.set_servo_pulsewidth(12, new_pulsewidth)
            self.esc1_pulsewidth = new_pulsewidth

        adjustment2 = self.esc2_axis_value * 10  # Smaller scale factor for smoother adjustment
        new_pulsewidth2 = max(1100, min(1900, self.esc2_pulsewidth + adjustment2))
        if new_pulsewidth != self.esc2_pulsewidth: #updates PWM value if joystick moved
            self.pi.set_servo_pulsewidth(13, new_pulsewidth2)
            self.esc2_pulsewidth = new_pulsewidth2
        
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        # Print ESC values in table format
        print(f"{timestamp:<20} | {self.esc1_pulsewidth:<17} | {self.esc2_pulsewidth}")

    def delay(self, seconds: float):
        time.sleep(seconds)

def main(args=None):
    rclpy.init(args=args)
    thruster_control = ThrusterControl()
    executor = MultiThreadedExecutor()
    rclpy.spin(thruster_control, executor=executor)
    thruster_control.pi.set_servo_pulsewidth(12, 1500)  # Reset to neutral on shutdown
    thruster_control.pi.set_servo_pulsewidth(13, 1500)  # Reset to neutral on shutdown
    thruster_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
