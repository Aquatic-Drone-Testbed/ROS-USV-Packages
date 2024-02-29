# from tokenize import String
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import pigpio
import time

class ThrusterControl(Node):

    #CONSTANTS FOR GPIO/ESC/THRUSTERS
    ESC_START_UP_VAL = 1500
    ESC_MAX_VAL = 1850
    ESC_MIN_VAL = 1150
    
    GPIO_ESC_PIN1 = 12
    GPIO_ESC_PIN2 = 13

    def __init__(self):
        super().__init__('thruster_control')
        
        self.subscription = self.create_subscription(
            String, 
            'thruster_control', 
            self.listener_callback,  #receives messages/ processes controller input
            10)
        
        try:            
            self.pi = pigpio.pi() # connect to pi gpio
            if not self.pi.connected:
                self.get_logger().error('Not connected to Raspberry Pi GPIO. Exiting...')
                rclpy.shutdown()
        except:
            self.get_logger().info('Not testing on Pi')
        
        self.leftESC_pulsewidth = ThrusterControl.ESC_START_UP_VAL 
        self.rightESC_pulsewidth = ThrusterControl.ESC_START_UP_VAL 
        
        time.sleep(0.7)
        
        self.pi.set_servo_pulsewidth(ThrusterControl.GPIO_ESC_PIN1, self.leftESC_pulsewidth)  # gpio pin is 12
        self.pi.set_servo_pulsewidth(ThrusterControl.GPIO_ESC_PIN2, self.rightESC_pulsewidth)  # gpio pin is 13
        
        self.timer = self.create_timer(0.1, self.timer_callback)  # Adjust PWM vaue at most every 0.1 seconds
        
        self.get_logger().info('Thruster control initialized.')
        print(f"{'Timestamp':<20} | {'Left ESC Pulsewidth':<17} | {'Right ESC Pulsewidth'}")
        print("-" * 60)  # Print a separator line

    def listener_callback(self, msg):
        #TODO get data and adjust thrusters 
        '''
            use msg value to update new_pulsewidth
        '''
        data_parts = msg.data.split(',')
        if data_parts[0] == "ABS_Y":
            self.leftESC_pulsewidth = self.ESC_START_UP_VAL + round(int(data_parts[1]))
            self.rightESC_pulsewidth = self.ESC_START_UP_VAL + round(int(data_parts[1]))
        if data_parts[0] == "ABS_RX":
            self.leftESC_pulsewidth = self.ESC_START_UP_VAL + round(int(data_parts[1]))
            self.rightESC_pulsewidth = self.ESC_START_UP_VAL - round(int(data_parts[1]))

        
    def timer_callback(self):
        # # Use the stored axis value to adjust PWM value
        # # adjustment = self.leftESC_axis_value * 10  # Smaller scale factor for smoother adjustment
        # new_pulsewidth = max(Thruster   Control.ESC_MIN_VAL, min(ThrusterControl.ESC_MAX_VAL, self.leftESC_pulsewidth + adjustment))
        # if new_pulsewidth != self.leftESC_pulsewidth: #updates PWM value if joystick moved
        self.pi.set_servo_pulsewidth(ThrusterControl.GPIO_ESC_PIN1, self.leftESC_pulsewidth)
        self.get_logger().info(f'Set Left ESC PWM value to {self.leftESC_pulsewidth}')
        #     self.leftESC_pulsewidth = new_pulsewidth

        # adjustment2 = self.rightESC_axis_value * 10  # Smaller scale factor for smoother adjustment
        # new_pulsewidth2 = max(ThrusterControl.ESC_MAX_VAL, min(ThrusterControl.ESC_MAX_VAL, self.rightESC_pulsewidth + adjustment2))
        # if new_pulsewidth != self.rightESC_pulsewidth: #updates PWM value if joystick moved
        self.pi.set_servo_pulsewidth(ThrusterControl.GPIO_ESC_PIN2, self.rightESC_pulsewidth)
        self.get_logger().info(f'Set Right ESC PWM value to {self.rightESC_pulsewidth}')
        #     self.rightESC_pulsewidth = new_pulsewidth2
        
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        # Print ESC values in table format
        print(f"{timestamp:<20} | Left ESC: {self.leftESC_pulsewidth:<17} | Right ESC: {self.rightESC_pulsewidth}")


def main(args=None):
    rclpy.init(args=args)
    thruster_control = ThrusterControl()
    executor = MultiThreadedExecutor()
    rclpy.spin(thruster_control, executor=executor)
    thruster_control.pi.set_servo_pulsewidth(ThrusterControl.GPIO_ESC_PIN1, ThrusterControl.ESC_START_UP_VAL)  # Reset to neutral on shutdown
    thruster_control.pi.set_servo_pulsewidth(ThrusterControl.GPIO_ESC_PIN2, ThrusterControl.ESC_START_UP_VAL)  # Reset to neutral on shutdown
    thruster_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
