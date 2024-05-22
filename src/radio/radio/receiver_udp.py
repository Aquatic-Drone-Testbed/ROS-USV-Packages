import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading

class UDPReceiver(Node):
    RADIO_TIMEOUT_SECONDS = 5
    TIMEOUT_MSG = f"TIMEOUT:{RADIO_TIMEOUT_SECONDS}"
    
    def __init__(self):
        super().__init__('udp_receiver_node')
        # Declare and obtain the UDP port as a parameter
        self.declare_parameter('port', 9000)  # Default port is 9000
        self.declare_parameter('host', "127.0.0.1")  # Default ip addr is local host
        
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.host, self.port))
        self.socket.settimeout(UDPReceiver.RADIO_TIMEOUT_SECONDS)

        # [TODO] for other publishers
        self.thruster_controller_publisher = self.create_publisher(String, "thruster_control", 10)
        self.camera_control_publisher = self.create_publisher(String, "camera_control", 10)
        self.radar_control_publisher = self.create_publisher(String, "radar_control", 10)
        self.radar_on = False
        self.thread = threading.Thread(target=self.listen, daemon=True)
        self.running = False

    def start(self):
        self.running = True
        self.thread.start()
        rclpy.spin(self)

    def listen(self):
        self.get_logger().info(f"UDP Receiver listening on {self.host}:{self.port}")
        
        while self.running:
            try:
                data, address = self.socket.recvfrom(1024)  # Buffer size is 1024 bytes
                message = data.decode()
            except socket.timeout as e:
                self.get_logger().warn(f"Socket timed out after {UDPReceiver.RADIO_TIMEOUT_SECONDS} seconds")
                message = UDPReceiver.TIMEOUT_MSG
            except Exception as e:
                self.get_logger().error(f"An error occurred while receiving data: {e}")
                # may want to break the loop or take other actions
                break  # or `continue` depending on your error handling strategy
            
            self.publish_data(message)

    def publish_data(self, message):
        # Assume data format "TYPE:data_value"
        data_type, data_value = message.split(':')
        
        msg = String()
        # self.get_logger().info(f'Received data: "{message}')
        self.get_logger().debug(f"Data type:-{data_type}-")
        self.get_logger().debug(f"Data Value:-{data_value}-")

        match data_type:
            case "TIMEOUT":
                msg.data = "RADIO_TIMEOUT"
                self.thruster_controller_publisher.publish(msg)
            case "CTRL":
                msg.data = data_value
                self.thruster_controller_publisher.publish(msg)
                self.get_logger().debug(f'Publishing to {"thruster_control"}: "{data_value}"')
            case "CAM TOGGLE":
                msg.data = data_value
                if(data_value == "CAM TOGGLE"):
                    self.camera_control_publisher.publish(msg)
                    self.get_logger().info(f'Publishing to camera_control: "{data_value}"')
            case "RADAR TOGGLE":
                if msg.data == "start_scan":
                    self.radar_on = True
                elif msg.data == "stop_scan":
                    self.radar_on = False
                self.radar_control_publisher.publish(msg)
                self.get_logger().info(f'Publishing to radar_control: {msg.data}')
            case "RADAR RANGE":
                msg.data = "cycle range"
                self.radar_control_publisher.publish(msg)
                self.get_logger().info(f'Publishing to radar_control: {msg.data}')
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
