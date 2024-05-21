import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import roslibpy

class PolarToRos1Node(Node):
    def __init__(self):
        super().__init__('polar_to_ros1_node')

        # Declare the 'host' parameter and get its value
        self.declare_parameter('host', 'localhost')  # default value is 'localhost'
        host = self.get_parameter('host').get_parameter_value().string_value

        # Subscribe to the ROS2 topic
        self.subscription = self.create_subscription(
            Image,
            '/USV/Polar',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Initialize the ROS1 client
        self.ros_client = roslibpy.Ros(host=host, port=9090) # `host` should be the host ip in ros1
        self.ros_client.run()
        
        if self.ros_client.is_connected:
            self.get_logger().info(f'Connected to ROS1 at {host}')
        else:
            self.get_logger().error(f'Failed to connect to ROS1 at {host}')
            return

        # Initialize the ROS1 topic
        self.ros_topic = roslibpy.Topic(self.ros_client, '/Navtech/Polar', 'sensor_msgs/Image')

    def listener_callback(self, msg):
        self.get_logger().info('Received message, sending to ROS1')
        
        # Convert ROS2 message to ROS1 format
        ros_msg = {
            'header': {
                'stamp': {
                    'secs': msg.header.stamp.sec,
                    'nsecs': msg.header.stamp.nanosec
                },
                'frame_id': msg.header.frame_id
            },
            'height': msg.height,
            'width': msg.width,
            'encoding': msg.encoding,
            'is_bigendian': msg.is_bigendian,
            'step': msg.step,
            'data': list(msg.data)
        }
        
        # Publish the message to ROS1
        self.ros_topic.publish(ros_msg)

    def destroy_node(self):
        self.get_logger().info('Shutting down node and terminating ROS1 connection')
        self.ros_topic.unadvertise()
        self.ros_client.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PolarToRos1Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
