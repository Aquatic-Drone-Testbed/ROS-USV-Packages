# from tokenize import String
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import pigpio
import time
import numpy as np

class ThrusterControl(Node):
    #CONSTANTS FOR GPIO/ESC/THRUSTERS
    ESC_BASE_VAL = 1500
    ESC_MAX_VAL = 1850
    ESC_MIN_VAL = 1150
    ESC_MAGNITUE = ESC_MAX_VAL - ESC_BASE_VAL

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

        self.base_thrust = 0
        self.delta_thrust = 0

        time.sleep(0.7)
        
        self.timer = self.create_timer(0.01, self.timer_callback)  # Adjust PWM vaue at most every 0.1 seconds
        
        self.get_logger().info('Thruster control initialized.')
        self.get_logger().info(f"{'Timestamp':<10} | {'Left ESC Pulsewidth':<17} | {'Right ESC Pulsewidth'}")
        self.get_logger().info("-" * 60)  # Print a separator line


    def listener_callback(self, msg):
        if msg.data == "RADIO_TIMEOUT":
            self.get_logger().warn('Radio timed out... Resetting thrusters to 0')
            self.base_thrust = 0
            self.delta_thrust = 0
            return
        
        #Adjust thruster values
        direction, value_str = msg.data.split(',')
        value = int(value_str)/ThrusterControl.ESC_MAGNITUE # normalize to [-1, 1]
        
        if direction == "ABS_Y":
            self.base_thrust = value
        elif direction == "ABS_X":
            self.delta_thrust = value


    def timer_callback(self):
        self.update_thrusters(self.base_thrust, self.delta_thrust)


    def update_thrusters(self, base, delta):
        left_val = ThrusterControl.ESC_BASE_VAL - ThrusterControl.ESC_MAGNITUE * (base + delta)
        right_val = ThrusterControl.ESC_BASE_VAL - ThrusterControl.ESC_MAGNITUE * (base - delta)

        # clamp between min and max value
        left_val = np.clip(round(left_val), ThrusterControl.ESC_MIN_VAL, ThrusterControl.ESC_MAX_VAL)
        right_val = np.clip(round(right_val), ThrusterControl.ESC_MIN_VAL, ThrusterControl.ESC_MAX_VAL)

        # Set GPIO PWM values
        self.pi.set_servo_pulsewidth(ThrusterControl.GPIO_ESC_PIN1, left_val)
        self.pi.set_servo_pulsewidth(ThrusterControl.GPIO_ESC_PIN2, right_val)

        timestamp = time.strftime("%H:%M:%S", time.localtime())
        self.get_logger().info(f"{timestamp:<10} | {f'Left ESC: {left_val}':<17} | Right ESC: {right_val}")


def main(args=None):

    rclpy.init(args=args)
    thruster_control = ThrusterControl()
    executor = MultiThreadedExecutor()
    rclpy.spin(thruster_control, executor=executor)
    thruster_control.pi.set_servo_pulsewidth(ThrusterControl.GPIO_ESC_PIN1, ThrusterControl.ESC_BASE_VAL)  # Reset to neutral on shutdown
    thruster_control.pi.set_servo_pulsewidth(ThrusterControl.GPIO_ESC_PIN2, ThrusterControl.ESC_BASE_VAL)  # Reset to neutral on shutdown
    thruster_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
