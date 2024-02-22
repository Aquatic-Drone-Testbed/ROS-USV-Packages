import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading

class UDPReceiver(Node):
    def __init__(self, host='', port=9000, topic='udp_data'):
        super().__init__('udp_receiver_node')
        self.publisher_ = self.create_publisher(String, topic, 10)
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.host, self.port))
        self.thread = threading.Thread(target=self.listen, daemon=True)
        self.running = False

    def start(self):
        self.running = True
        self.thread.start()
        rclpy.spin(self)

    def listen(self):
        self.get_logger().info(f"UDP Receiver listening on {self.host}:{self.port}")
        while self.running:
            data, _ = self.socket.recvfrom(1024)  # Buffer size is 1024 bytes
            message = data.decode()
            self.publish_data(message)

    def publish_data(self, data):
        # Assume data format "TYPE:ActualData"
        data_type, _, actual_data = data.partition(':')
        
        if data_type == "KEY":
            topic = "thruster_control"
        elif data_type == "VIDEO":
            topic = "video_stream"
        elif data_type == "RADAR":
            topic = "radar_data"
        else: #add other keyboard or controller commands 
            self.get_logger().error(f"Unknown data type: {data_type}")
            return

        # Assume all topics use std_msgs/msg/String for simplicity
        msg = String()
        msg.data = actual_data
        # Dynamically select publisher based on data type
        publisher = self.get_publisher_for_topic(topic)
        if publisher:
            publisher.publish(msg)
            self.get_logger().info(f'Publishing to {topic}: "{actual_data}"')
        else:
            self.get_logger().error(f"No publisher available for topic: {topic}")


    def stop(self):
        self.running = False
        self.socket.close()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    udp_port = 9000  # Adjust the port as needed
    topic = 'udp_data'  # The ROS topic to publish UDP data to
    udp_receiver = UDPReceiver(port=udp_port, topic=topic)
    try:
        udp_receiver.start()
    except KeyboardInterrupt:
        pass
    finally:
        udp_receiver.stop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
