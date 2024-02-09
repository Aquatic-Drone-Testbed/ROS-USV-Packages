import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoystickListener(Node):
    def __init__(self):
        super().__init__('joystick_listener')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)

    def joy_callback(self, msg):
        # Process joystick input here
        self.get_logger().info('Joystick Input: Axes: "%s", Buttons: "%s"' % (msg.axes, msg.buttons))

def main(args=None):
    rclpy.init(args=args)
    joystick_listener = JoystickListener()
    rclpy.spin(joystick_listener)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
