# from tokenize import String
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import lgpio
import time
import numpy as np

class ThrusterControl(Node):
    #CONSTANTS FOR GPIO/ESC/THRUSTERS
    ESC_BASE_VAL = 1500
    ESC_MAX_VAL = 1850
    ESC_MIN_VAL = 1150
    ESC_MAGNITUDE = ESC_MAX_VAL - ESC_BASE_VAL

    GPIO_ESC_PIN1 = 12
    GPIO_ESC_PIN2 = 13

    def __init__(self):
        super().__init__('thruster_control_node')
        self.base_thrust = 0
        self.delta_thrust = 0
        self.subscription = self.create_subscription(
            String, 
            'thruster_input', 
            self.update_pwm,  #receives messages/ processes controller input``
            10)
        
        self.pi = lgpio.gpiochip_open(4) # connect to pi gpio
        if self.pi < 0:
            self.get_logger().error('Not connected to Raspberry Pi GPIO. Exiting...')
            rclpy.shutdown()

        lgpio.gpio_claim_output(self.pi, ThrusterControl.GPIO_ESC_PIN1)
        lgpio.gpio_claim_output(self.pi, ThrusterControl.GPIO_ESC_PIN2)

        print(f"self.pi = {self.pi}")

        print(lgpio.gpio_get_chip_info(self.pi))
        print(lgpio.gpio_get_line_info(self.pi,ThrusterControl.GPIO_ESC_PIN1))
        print(lgpio.gpio_get_line_info(self.pi,ThrusterControl.GPIO_ESC_PIN2))

        #time.sleep(0.7)
        
        #self.timer = self.create_timer(0.01, self.update_pwm)  # update pwm 100 times per second
        
        self.get_logger().info('Thruster control initialized.')
        # self.get_logger().info(f"{'Timestamp':<10} | {'Left ESC Pulsewidth':<17} | {'Right ESC Pulsewidth'}")
        self.get_logger().info("-" * 60)  # Print a separator line




    def update_pwm(self, msg):
        split_mes = msg.data.split(",")
        self.base_thrust = float(split_mes[0])
        self.delta_thrust = float(split_mes[1])
        left_val = ThrusterControl.ESC_BASE_VAL + ThrusterControl.ESC_MAGNITUDE * (self.base_thrust + self.delta_thrust)
        right_val = ThrusterControl.ESC_BASE_VAL + ThrusterControl.ESC_MAGNITUDE * (self.base_thrust - self.delta_thrust)

        # clamp between min and max value
        left_val = np.clip(round(left_val), ThrusterControl.ESC_MIN_VAL, ThrusterControl.ESC_MAX_VAL)
        right_val = np.clip(round(right_val), ThrusterControl.ESC_MIN_VAL, ThrusterControl.ESC_MAX_VAL)

        print(f"{int(left_val)}-{int(right_val)}")

        # Set GPIO PWM values

        #lgpio.tx_pwm(self.pi,ThrusterControl.GPIO_ESC_PIN1,50,7.75)
        #lgpio.tx_pwm(self.pi,ThrusterControl.GPIO_ESC_PIN2,50,7.75)

        lgpio.tx_servo(self.pi,ThrusterControl.GPIO_ESC_PIN1,int(left_val))
        lgpio.tx_servo(self.pi,ThrusterControl.GPIO_ESC_PIN2,int(right_val))

        #self.pi.set_servo_pulsewidth(ThrusterControl.GPIO_ESC_PIN1, left_val)
        #self.pi.set_servo_pulsewidth(ThrusterControl.GPIO_ESC_PIN2, right_val)

        self.get_logger().debug(f"{self.base_thrust = } {self.delta_thrust = } {left_val = } {right_val = }")


def main(args=None):

    rclpy.init(args=args)
    thruster_control = ThrusterControl()
    executor = MultiThreadedExecutor()
    rclpy.spin(thruster_control, executor=executor)
    #thruster_control.pi.set_servo_pulsewidth(ThrusterControl.GPIO_ESC_PIN1, ThrusterControl.ESC_BASE_VAL)  # Reset to neutral on shutdown
    #thruster_control.pi.set_servo_pulsewidth(ThrusterControl.GPIO_ESC_PIN2, ThrusterControl.ESC_BASE_VAL)  # Reset to neutral on shutdown
    thruster_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
