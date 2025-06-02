#TCP RADAR SPOKE VERSION
import socket
import io

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy

from cv_bridge import CvBridge
from PIL import Image as PilImage

import zlib
import struct
import re

class RadioServer(Node):
    RADIO_TIMEOUT_SECONDS = 2.5
    USV_SERVER_PORT = 39000
    GPS_DATA_PORT = 39001
    VIDEO_STREAM_PORT = 39002
    RADAR_STREAM_PORT = 39003
    LIDAR_STREAM_PORT = 39006
    DIAGNOSTIC_PORT = 39004
    SLAM_PORT = 39005 #Using this to send radar scan string data
    SPOKE_PORT = 39007
    REQUEST_CONNECTION_STR = 'Ping'
    ACKNOWLEDGE_CONNECTION_STR = 'Pong'
        
    def __init__(self):
        super().__init__('radio_server_node')
        
        # ros params
        self.declare_parameter('port', RadioServer.USV_SERVER_PORT)
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', self.port))
        self.sock.settimeout(RadioServer.RADIO_TIMEOUT_SECONDS)
        
        self.sock_tcp = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

        self.get_logger().info(f"Radio server listening on UDP port {self.port}...")
        
        reentrant_callback_group = ReentrantCallbackGroup()
        mutex_callback_group = MutuallyExclusiveCallbackGroup()

        self.last_spoke = -1
        
        # Adjust these topic names and types according to your actual topics and data types
        gps_data_qos = video_stream_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # request data from radio asap
        self.data_timer = self.create_timer(
            0, 
            self.recv_from_ctrl_station,
            mutex_callback_group)
        
        # publishers
        self.thruster_controller_publisher = self.create_publisher(
            String, 
            "thruster_control", 
            10,
            callback_group=reentrant_callback_group)
        self.camera_control_publisher = self.create_publisher(
            String, 
            "camera_control", 
            10,
            callback_group=reentrant_callback_group)
        self.radar_control_publisher = self.create_publisher(
            String, 
            "radar_control", 
            10,
            callback_group=reentrant_callback_group)
        self.navmod_control_publisher = self.create_publisher(
            String, 
            "navmod_control", 
            10,
            callback_group=reentrant_callback_group)
        
        # subscribers
        self.create_subscription(
            Image, 
            'video_stream', 
            self.video_stream_callback, 
            video_stream_qos,
            callback_group=reentrant_callback_group)
        self.create_subscription(
            Image, 
            'radar_image', 
            self.radar_stream_callback, 
            video_stream_qos,
            callback_group=reentrant_callback_group)
        self.create_subscription(
            Image, 
            'lidar_image', 
            self.lidar_stream_callback, 
            video_stream_qos,
            callback_group=reentrant_callback_group)
        self.create_subscription(
            NavSatFix, 
            'gps_data', 
            self.gps_data_callback, 
            gps_data_qos,
            callback_group=reentrant_callback_group)
        self.create_subscription(
            String, 
            'radar_spoke', 
            self.radar_spoke_callback, 
            10,
            callback_group=reentrant_callback_group)
        self.create_subscription(
            String, 
            'radar_scan_str', 
            self.radar_scan_callback, 
            10,
            callback_group=reentrant_callback_group)
        self.create_subscription(
            String, 
            'diagnostic_status', 
            self.diagnostics_callback, 
            10,
            callback_group=reentrant_callback_group)
        
        self.control_station_ip = None
        self.control_station_port = None
        self.radar_scanning = False
        self.bridge = CvBridge()# Create a CvBridge object to convert between ROS Image messages and OpenCV images


    def recv_from_ctrl_station(self):
        try:
            data, (self.control_station_ip, self.control_station_port) = self.sock.recvfrom(1024)  # Buffer size is 1024 bytes
            self.get_logger().debug(f'Received {data} from {self.control_station_ip}:{self.control_station_port}')
        except socket.timeout as e:
            self.get_logger().warn(f"Socket receive timed out after {RadioServer.RADIO_TIMEOUT_SECONDS} seconds")
            self.process_radio_timeout()
            return
        except Exception as e:
            self.get_logger().error(f"An error occurred while receiving data: {e}")
            return
        
        self.process_control_station_data(data)


    def process_radio_timeout(self):
        msg = String()
        msg.data = "RADIO_TIMEOUT"
        self.thruster_controller_publisher.publish(msg)


    def process_control_station_data(self, data):
        data_str = data.decode()
        if data_str == RadioServer.REQUEST_CONNECTION_STR:
            connected = False
            self.get_logger().info(f'Received {RadioServer.REQUEST_CONNECTION_STR} from control station ({self.control_station_ip}:{self.control_station_port})')
            self.get_logger().info(f'Connecting TCP...')
            while not connected:
                try:
                    self.sock_tcp.connect((self.control_station_ip,RadioServer.SPOKE_PORT))
                    self.get_logger().info(f'TCP Connected!...')
                    connected = True
                except ConnectionError:
                    self.get_logger().info(f'Retrying TCP Connection to {self.control_station_ip}...')
                except OSError:
                    self.sock_tcp.close()
                    self.sock_tcp = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
                except KeyboardInterrupt:
                    break
            self.get_logger().info(f'Sending {RadioServer.ACKNOWLEDGE_CONNECTION_STR} back to control station')
            self.send_to_ctrl_station(RadioServer.ACKNOWLEDGE_CONNECTION_STR.encode(), self.control_station_port)
            return
        
        try: # Assume data_str format is "data_type:data_value"
            data_type, data_value = data_str.split(':')
        except ValueError as e:
            self.get_logger().error(f'Failed to parse {data_str} from control station')
            return
        self.get_logger().debug(f"{data_type=}")
        self.get_logger().debug(f"{data_value=}")
        
        msg = String()
        
        match data_type:
            case "ctrl":
                msg.data = data_value # special case
                self.thruster_controller_publisher.publish(msg)
            case "cam":
                if data_value == "toggle_cam":
                    msg.data = "CAM TOGGLE"
                self.camera_control_publisher.publish(msg)
            case "radar":
                if data_value == "toggle_scan":
                    msg.data = "toggle_scan"
                elif data_value == "zoom_in":
                    msg.data = "zoom_in"
                elif data_value == "zoom_out":
                    msg.data = "zoom_out"
                self.radar_control_publisher.publish(msg)
            case "navmod":
                if data_value == "toggle_navmod":
                    msg.data = "toggle_navmod"
                self.navmod_control_publisher.publish(msg)
            case _:
                self.get_logger().error(f"Unknown data type: {data_type}")


    def video_stream_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough') #Convert MSG to OpenCV/CV_Bridge image format
        pil_image = PilImage.fromarray(cv_image) # Convert to Pillow image (allows us to avoid using openCV)
        # Compress the image as JPEG
        buffer = io.BytesIO()
        pil_image.save(buffer, format='JPEG', quality=80)  # Adjust the quality as needed
        compressed_img = buffer.getvalue()
        self.send_to_ctrl_station(compressed_img, RadioServer.VIDEO_STREAM_PORT)


    def radar_stream_callback(self, msg: Image):
        pass
#        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#        pil_image = PilImage.fromarray(cv_image)
#        buffer = io.BytesIO()
#        pil_image.save(buffer, format='JPEG', quality=80)
#        compressed_img = buffer.getvalue()
#        self.send_to_ctrl_station(compressed_img, RadioServer.RADAR_STREAM_PORT)


    def lidar_stream_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        pil_image = PilImage.fromarray(cv_image)
        buffer = io.BytesIO()
        pil_image.save(buffer, format='JPEG', quality=80)
        compressed_img = buffer.getvalue()
        self.send_to_ctrl_station(compressed_img, RadioServer.LIDAR_STREAM_PORT)


    def gps_data_callback(self, msg: NavSatFix):
        gps_data = f"Latitude: {msg.latitude}, Longitude: {msg.longitude}, Altitude: {msg.altitude}".encode()
        self.send_to_ctrl_station(gps_data, RadioServer.GPS_DATA_PORT)

    def radar_spoke_callback(self, msg: String):
        match = re.search(r'Q_Header<\((.*?)\)>',msg.data)
        if match: 
            q_header = match.group(1)
            elements = [x for x in q_header.split(',')]
            if int(elements[7]) != self.last_spoke+1 and int(elements[7]) != 0:
                print(f'Dropped Spoke! Last: {self.last_spoke} Current: {int(elements[7])}')
            self.last_spoke = int(elements[7])
        
        #radar_spoke_data = zlib.compress(msg.data.encode()) #FOR COMPRESSION
        radar_spoke_data = msg.data.encode() #FOR NO COMPRESSION
        length = len(radar_spoke_data) #for TCP
        radar_spoke_data = struct.pack('!I', length) + radar_spoke_data
        self.get_logger().debug(f'Sent {msg} to station via port {RadioServer.SPOKE_PORT}')
        #print("SENT SPOKE")
        
        #self.send_to_ctrl_station(radar_spoke_data, RadioServer.SPOKE_PORT) #SEND VIA UDP

        self.send_to_ctrl_station_tcp(radar_spoke_data) #SEND VIA TCP (MUST REMOVE TCP CONNECT CODE IF COMMENTED OUT TO USE UDP)

    def radar_scan_callback(self, msg: String):
        pass
        #uncomment to send full radar scans (note: currently packets too large)
        #radar_scan_data = msg.data.encode()
        #self.get_logger().debug(f'Sent {msg} to station via port {RadioServer.SLAM_PORT}')
        #self.send_to_ctrl_station(radar_scan_data, RadioServer.SLAM_PORT)

    def diagnostics_callback(self, msg: String):
        diagnostic_data = msg.data.encode()
        #diagnostic_data = zlib.compress(msg.data.encode())
        self.get_logger().debug(f'Sent {msg} to station via port {RadioServer.DIAGNOSTIC_PORT}')
        self.send_to_ctrl_station(diagnostic_data, RadioServer.DIAGNOSTIC_PORT)

    def send_to_ctrl_station(self, data: bytes, port):
        if self.control_station_ip is None:
            self.get_logger().warn(f'Control station not connected!')
            return
        
        self.sock.sendto(data, (self.control_station_ip, port))
        self.get_logger().debug(f'Sent {len(data)} bytes to {self.control_station_ip}:{self.control_station_port}')

    def send_to_ctrl_station_tcp(self, data: bytes):
        if self.control_station_ip is None:
            self.get_logger().warn(f'[TCP] Control station not connected!')
            return
        
        try:
            self.sock_tcp.send(data)
            self.get_logger().debug(f'[TCP] Sent {len(data)} bytes to {self.control_station_ip}:{RadioServer.SPOKE_PORT}')
        except OSError:
            print("TCP Send Failed")


def main(args=None):
    rclpy.init(args=args)
    udp_receiver = RadioServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(udp_receiver)
    executor.spin()
    udp_receiver.destroy_node()
    executor.shutdown()


if __name__ == '__main__':
    main()
