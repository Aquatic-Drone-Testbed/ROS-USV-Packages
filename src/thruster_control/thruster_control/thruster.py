from tokenize import String
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import pigpio
import time

class ThrusterControl(Node):

    #CONSTANTS FOR GPIO/ESC/THRUSTERS
    ESC_START_UP_VAL = 1500
    ESC_MAX_VAL = 1900
    ESC_MIN_VAL = 1100
    
    GPIO_ESC_PIN1 = 12
    GPIO_ESC_PIN2 = 13

    def __init__(self):
        super().__init__('thruster_control')
        
        self.subscription = self.create_subscription(
            String, 
            'thruster_control', 
            self.listener_callback,  #receives messages/ processes controller input
            10)
                    
        self.pi = pigpio.pi() # connect to pi gpio
        if not self.pi.connected:
            self.get_logger().error('Not connected to Raspberry Pi GPIO. Exiting...')
            rclpy.shutdown()

        self.esc1_pulsewidth = ESC_START_UP_VAL 
        self.esc2_pulsewidth = ESC_START_UP_VAL 
        
        time.sleep(0.7)
        
        self.esc1_axis_value = 0  # Gets and stores latest joystick position
        self.esc2_axis_value = 0
        
        self.pi.set_servo_pulsewidth(GPIO_ESC_PIN1, self.esc1_pulsewidth)  # gpio pin is 12
        self.pi.set_servo_pulsewidth(GPIO_ESC_PIN2, self.esc2_pulsewidth)  # gpio pin is 13
        
        self.timer = self.create_timer(0.1, self.timer_callback)  # Adjust PWM vaue at most every 0.1 seconds
        
        self.get_logger().info('Thruster control initialized.')
        print(f"{'Timestamp':<20} | {'ESC 1 Pulsewidth':<17} | {'ESC 2 Pulsewidth'}")
        print("-" * 60)  # Print a separator line

    def listener_callback(self, msg):
        #TODO get data and adjust thrusters 
            '''
                use msg value to update new_pulsewidth
            '''
        # Sets axis to be vertical joystick axis only
        self.esc1_axis_value = msg.axes[1]
        self.esc2_axis_value = msg.axes[4]

    def timer_callback(self):
        # Use the stored axis value to adjust PWM value
        adjustment = self.esc1_axis_value * 10  # Smaller scale factor for smoother adjustment
        new_pulsewidth = max(ESC_MIN_VAL, min(ESC_MAX_VAL, self.esc1_pulsewidth + adjustment))
        if new_pulsewidth != self.esc1_pulsewidth: #updates PWM value if joystick moved
            self.pi.set_servo_pulsewidth(GPIO_ESC_PIN1, new_pulsewidth)
            self.esc1_pulsewidth = new_pulsewidth

        adjustment2 = self.esc2_axis_value * 10  # Smaller scale factor for smoother adjustment
        new_pulsewidth2 = max(ESC_MAX_VAL, min(ESC_MAX_VAL, self.esc2_pulsewidth + adjustment2))
        if new_pulsewidth != self.esc2_pulsewidth: #updates PWM value if joystick moved
            self.pi.set_servo_pulsewidth(GPIO_ESC_PIN2, new_pulsewidth2)
            self.esc2_pulsewidth = new_pulsewidth2
        
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        # Print ESC values in table format
        print(f"{timestamp:<20} | {self.esc1_pulsewidth:<17} | {self.esc2_pulsewidth}")


def main(args=None):
    rclpy.init(args=args)
    thruster_control = ThrusterControl()
    executor = MultiThreadedExecutor()
    rclpy.spin(thruster_control, executor=executor)
    thruster_control.pi.set_servo_pulsewidth(GPIO_ESC_PIN1, ESC_START_UP_VAL)  # Reset to neutral on shutdown
    thruster_control.pi.set_servo_pulsewidth(GPIO_ESC_PIN2, ESC_START_UP_VAL)  # Reset to neutral on shutdown
    thruster_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
