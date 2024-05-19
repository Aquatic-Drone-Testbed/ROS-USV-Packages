import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import roslibpy

class PolarToRos1Node(Node):
    def __init__(self):
        super().__init__('polar_to_ros1_node')
        self.subscription = self.create_subscription(
            Image,
            '/USV/Polar',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.ros_client = roslibpy.Ros(host='localhost', port=9090)
        self.ros_client.run()
        self.ros_topic = roslibpy.Topic(self.ros_client, '/navtech_polar', 'sensor_msgs/Image')

    def listener_callback(self, msg):
        self.get_logger().info('Received message, sending to ROS1')
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
        self.ros_topic.publish(ros_msg)

    def destroy_node(self):
        self.ros_topic.unadvertise()
        self.ros_client.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PolarToRos1Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#colcon build --symlink-install --packages-select polar_to_ros1