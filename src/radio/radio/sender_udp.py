import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import socket
import cv2 
from cv_bridge import CvBridge

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy

class UDPSender(Node):
    def __init__(self):
        super().__init__('udp_sender_node')
        
        # Declare and get the target IP address as a parameter
        self.declare_parameter('target_ip', '127.0.0.1')  # Default to localhost
        target_ip_param = self.get_parameter('target_ip').get_parameter_value().string_value
        self.target_ip = target_ip_param
        
        # Create a CvBridge object to convert between ROS Image messages and OpenCV images
        self.bridge = CvBridge()
        
        
        # Adjust these topic names and types according to your actual topics and data types
        gps_data_qos = video_stream_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
     
        self.create_subscription(Image, 'video_stream', self.video_stream_callback, video_stream_qos)
        self.create_subscription(String, 'gps_data', self.gps_data_callback, gps_data_qos)
        
        # UDP target IP and port
        #adjust ports as needed
        self.video_stream_port = 9001
        self.gps_data_port = 9000

        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Send data to the target IP and port
    def send_udp_data(self, data, port):
        # If data is a string, encode it to bytes
        if isinstance(data, str):
            data = data.encode()
        self.udp_socket.sendto(data, (self.target_ip, port))
        self.get_logger().info(f'Sent data to {self.target_ip}:{port}')

    def video_stream_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # Compress the image as JPEG to reduce size, we can change this whatever later
        compressed_img = cv2.imencode('.jpg', cv_image)[1].tobytes()
        self.send_udp_data(compressed_img, self.video_stream_port)

    def gps_data_callback(self, msg):
        self.send_udp_data(msg.data, self.gps_data_port)

def main(args=None):
    rclpy.init(args=args)
    udp_sender_node = UDPSender()
    rclpy.spin(udp_sender_node)
    udp_sender_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
