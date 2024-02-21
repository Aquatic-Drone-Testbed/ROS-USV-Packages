import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket

class UDPSender(Node):
    def __init__(self):
        super().__init__('udp_sender_node')
        
        # Declare and get the target IP address as a parameter
        self.declare_parameter('target_ip', '127.0.0.1')  # Default to localhost
        target_ip_param = self.get_parameter('target_ip').get_parameter_value().string_value
        self.target_ip = target_ip_param
        
        # Adjust these topic names and types according to your actual topics and data types
        self.create_subscription(String, 'video_stream', self.video_stream_callback, 10)
        self.create_subscription(String, 'gps_data', self.gps_data_callback, 10)
        
        # UDP target IP and port
        #adjust ports as needed
        self.video_stream_port = 9876
        self.gps_data_port = 9877
        
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_udp_data(self, data, port):
        self.udp_socket.sendto(data.encode(), (self.target_ip, port))
        self.get_logger().info(f'Sent data to {self.target_ip}:{port}')

    def video_stream_callback(self, msg):
        self.send_udp_data(msg.data, self.video_stream_port)

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
