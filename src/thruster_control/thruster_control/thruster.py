# from tokenize import String
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rpi_hardware_pwm import HardwarePWM
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
        # make sure chip does not flip between pwmchip0 and pwmchip1
        # check ls -la /sys/class/pwm/pwmchip0
        # if the path is /devices/platform/soc/ this is the wrong chip
        # it should be /devices/platform/axi/ 
        # try pwmchip1 if it flips
        self.pi_1 = HardwarePWM(pwm_channel=0, hz=50, chip=0) # connect to pi gpio pin 12
        self.pi_2 = HardwarePWM(pwm_channel=1, hz=50, chip=0) # connect to pi gpio pin 13

        self.pi_1.start(7.5)
        self.pi_2.start(7.5)
        
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

        #print(f"{int(left_val)}-{int(right_val)}")

        #Calculate duty cycle
        left_val_dc = float(left_val)/20000 * 100 #20000 for 50 Hz
        right_val_dc = float(right_val)/20000 * 100 

        # Set GPIO PWM values

        self.pi_1.change_duty_cycle(left_val_dc)
        self.pi_2.change_duty_cycle(right_val_dc)

        self.get_logger().debug(f"{self.base_thrust = } {self.delta_thrust = } {left_val = } {right_val = }")


def main(args=None):

    rclpy.init(args=args)
    thruster_control = ThrusterControl()
    executor = MultiThreadedExecutor()
    rclpy.spin(thruster_control, executor=executor)
    thruster_control.pi_1.stop()  # Reset to neutral on shutdown
    thruster_control.pi_2.stop()  # Reset to neutral on shutdown
    thruster_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
