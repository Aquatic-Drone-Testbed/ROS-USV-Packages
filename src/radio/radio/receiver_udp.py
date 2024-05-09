import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading

class UDPReceiver(Node):
    UDP_TIMEOUT = 5
    
    def __init__(self):
        super().__init__('udp_receiver_node')
        # Declare and obtain the UDP port as a parameter
        self.declare_parameter('port', 9000)  # Default port is 9000
        self.declare_parameter('host', "127.0.0.1")  # Default ip addr is local host
        
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.host, self.port))
        self.socket.settimeout(10)
        
        ready = select.select([self.mysocket], [], [], UDPReceiver.UDP_TIMEOUT)

        # [TODO] for other publishers
        self.thruster_controller_publisher = self.create_publisher(String, "thruster_control", 10)
        
        self.thread = threading.Thread(target=self.listen, daemon=True)
        self.running = False

    def start(self):
        self.running = True
        self.thread.start()
        rclpy.spin(self)

    def listen(self):
        self.get_logger().info(f"UDP Receiver listening on {self.host}:{self.port}")
        self.get_logger().info(f"UDP Receiver listening on {self.host}:{self.port}")
        while self.running:
            try:
                data, _ = self.socket.recvfrom(1024)  # Buffer size is 1024 bytes
                message = data.decode()
                self.publish_data(message)
            except Exception as e:
                self.get_logger().error(f"An error occurred while receiving data: {e}")
                # may want to break the loop or take other actions
                break  # or `continue` depending on your error handling strategy

    def publish_data(self, data):
        # Assume data format "TYPE:ActualData"
        data_type, _, actual_data = data.partition(':')
        
        msg = String()
        msg.data = actual_data
        #TODO add other keyboard or controller commands 
        match data_type:
            case "CTRL":
                self.thruster_controller_publisher.publish(msg)
                self.get_logger().info(f'Publishing to {"thruster_control"}: "{actual_data}"')
            case _:
                self.get_logger().error(f"Unknown data type: {data_type}")

    def stop(self):
        self.running = False
        self.socket.close()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    udp_receiver = UDPReceiver()
    try:
        udp_receiver.start()
    except KeyboardInterrupt:
        pass
    finally:
        udp_receiver.stop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
