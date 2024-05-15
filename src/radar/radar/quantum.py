import socket
import struct
import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from sensor_msgs.msg import Image

from radar.packets.location_info import LocationInfo
from radar.packets import control_message
from radar.packets.rmr_report import RMReport
from radar.packets.quantum_scan import QuantumScan
from radar.packets.quantum_report import QuantumReport

MAX_SPOKE_LENGTH = 256
MAX_INTENSITY = 128
MAX_SPOKE_COUNT = 250
class FrameId:
    RM_REPORT = 0x00010001
    QUANTUM_REPORT = 0x00280002
    QUANTUM_SPOKE = 0x00280003

class Qauntum(Node):

    def __init__(self):
        super().__init__('quantum')
        # uncomment below to simulate locator
        # data = b'\x00\x00\x00\x00\x92\x8b\x80\xcb(\x00\x00\x00\x03\x00d\x00\x06\x08\x10\x00\x01\xb3\x01\xe8\x0e\n\x11\x002\x00\xe0\n\x0f\n6\x00'
        # bl = LocationInfo.parse(data[:36])
        # self.quantum_location = LocationInfo(*bl)
        self.quantum_location = self.locate_quantum() # this might take a while to return
        self.declare_parameter('host', "127.0.0.1")  # Default ip addr is local host
        self.host = self.get_parameter('host').get_parameter_value().string_value
        
        self.command_socket = self.create_command_socket() # socket for controlling radar
        self.report_socket = self.create_report_socket() # socket for receiving radar data
        
        reentrant_callback_group = ReentrantCallbackGroup()
        command_mutex_callback_group_ = MutuallyExclusiveCallbackGroup()
        receiver_mutex_callback_group = MutuallyExclusiveCallbackGroup()
        
        # timers
        # keep radar alive every 1 second
        self.standby_timer = self.create_timer(
            1, 
            self.standby_timer_callback, 
            command_mutex_callback_group_)
        # request polar image every 3 second
        self.polar_image_timer = self.create_timer(
            3, 
            self.imager_callback, 
            reentrant_callback_group)
        # request data from radar asap
        self.data_timer = self.create_timer(
            0, 
            self.radar_data_callback, 
            receiver_mutex_callback_group)
        
        # publisher
        self.image_publisher_ = self.create_publisher(
            Image, 
            'radar_image', 
            10, 
            callback_group=reentrant_callback_group)
        
        # subscription
        self.radar_control_subscription = self.create_subscription(
            String,
            'radar_control',
            self.radar_control_callback,
            10,
            callback_group=command_mutex_callback_group_)
        
        self.alive_counter = 0
        self.spokes = np.random.uniform(low=0, high=MAX_INTENSITY, size=(250, MAX_SPOKE_LENGTH))
        self.spokes_updated = 0
        self.bridge = CvBridge()


    def locate_quantum(self, multicast_group='224.0.0.1', multicast_port=5800, quantum_model_id=40) -> LocationInfo:
        # Create UDP socket
        locator_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP)
        locator_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Bind to the server address
        # on this port, receives ALL multicast groups
        locator_socket.bind(('', multicast_port))
        
        # Tell the operating system to add the socket to the multicast group
        # on all interfaces.
        mreq = struct.pack('4sL', socket.inet_aton(multicast_group), socket.INADDR_ANY)
        locator_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        self.get_logger().info(f'Initialized locator socket')
        
        # loop until Raymarine Quantum is found
        while True:
            self.get_logger().info(f'Locating radar...')
            data, senderaddr = locator_socket.recvfrom(1024)
            self.get_logger().debug(f'received {len(data)} bytes from {senderaddr[0]}:{senderaddr[1]}')
            if len(data) != 36: continue # ignore any packets not 36 bytes
            
            quantum_location = LocationInfo(*LocationInfo.parse(data))
            self.get_logger().debug(f'{quantum_location=}')
            if quantum_location.model_id != quantum_model_id: continue
            
            self.get_logger().info(f'Found radar at {quantum_location.radar_ip}')
            break
        
        self.get_logger().info(f'Closing locator socket')
        locator_socket.close()
        return quantum_location


    def create_command_socket(self) -> socket:
        command_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP)
        command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        command_socket.setblocking(0) # non-blocking socket
        self.get_logger().info(f'Initialized command socket')
        
        return command_socket


    def standby_timer_callback(self):
        self.radar_stay_alive()
        self.alive_counter = (self.alive_counter + 1)%5


    def radar_stay_alive(self) -> None:
        self.transmit_command(control_message.STAY_ALIVE_1SEC)
        if (self.alive_counter%5 == 0):
            self.transmit_command(control_message.STAY_ALIVE_5SEC)
        self.get_logger().debug(f'Sent stay alive command')


    def transmit_command(self, command) -> None:
        self.command_socket.sendto(command, (self.quantum_location.radar_ip, self.quantum_location.radar_port))
        self.get_logger().debug(f'Sent {len(command)} bytes to {self.quantum_location.radar_ip}:{self.quantum_location.radar_port}')


    def start_scan(self) -> None:
        self.transmit_command(control_message.TX_ON)
        self.get_logger().info(f'Sent tx on command')


    def stop_scan(self) -> None:
        self.transmit_command(control_message.TX_OFF)
        self.get_logger().info(f'Sent tx off command')


    def create_report_socket(self) -> socket:
        report_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP)
        report_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Bind to the server address
        # on this port, receives ALL multicast groups
        report_socket.bind(('', self.quantum_location.data_port))
        # Tell the operating system to add the socket to the multicast group
        # on all interfaces.
        # mreq = struct.pack('4sL', socket.inet_aton(self.quantum_location.data_ip), socket.INADDR_ANY) # no needed
        self.get_logger().info(f'receiver host: {self.host}')
        mreq = struct.pack('4s4s', socket.inet_aton(self.quantum_location.data_ip), socket.inet_aton(self.host)) # set to the ip of the receiver
        report_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        self.get_logger().info(f'Initialized report socket')
        
        return report_socket


    def radar_data_callback(self):
        try:
            data, senderaddr = self.report_socket.recvfrom(1024)
        except OSError:
            return None
        
        self.process_frame(data)


    def process_frame(self, data: bytes):
        if len(data) < 4: return # data must be longer than 4 bytes
        
        self.last_frame_id = struct.unpack('<I', data[:4])[0] # read first 4 bytes
        self.get_logger().debug(f'Received {len(data)} bytes (frame_id=0x{self.last_frame_id:X})')
        self.get_logger().debug(f'{data}')
        
        match self.last_frame_id:
            case FrameId.RM_REPORT:
                self.process_rm_report(data)
                pass
            case 0x00010002:
                # ProcessFixedReport(data, len)
                pass
            case 0x00010003:
                # ProcessScanData(data, len)
                pass
            case FrameId.QUANTUM_SPOKE:
                self.process_quantum_scan_data(data)
            case FrameId.QUANTUM_REPORT:
                self.process_quantum_report(data)
            case 0x00280001:  # type and serial for Quantum radar
                pass
                # IF_serial = wxString::FromAscii(data + 10, 7)
                # MOD_serial = wxString::FromAscii(data + 4, 6)
                # if (MOD_serial == _('E70498')) {
                # m_ri->m_quantum2type = true
                # }
                # m_ri->m_radar_location_info.serialNr = IF_serial
                # status = m_ri->m_state.GetValue()

                # match status:
                #     case RADAR_OFF:
                #         LOG_VERBOSE(wxT('%s reports status RADAR_OFF'), m_ri->m_name.c_str())
                #         stat = _('Off')
                #     case RADAR_STANDBY:
                #         LOG_VERBOSE(wxT('%s reports status STANDBY'), m_ri->m_name.c_str())
                #         stat = _('Standby')
                #     case RADAR_WARMING_UP:
                #         LOG_VERBOSE(wxT('%s reports status RADAR_WARMING_UP'), m_ri->m_name.c_str())
                #         stat = _('Warming up')
                #     case RADAR_TRANSMIT:
                #         LOG_VERBOSE(wxT('%s reports status RADAR_TRANSMIT'), m_ri->m_name.c_str())
                #         stat = _('Transmit')
                #     case _:
                #         # LOG_BINARY_RECEIVE(wxT('received unknown radar status'), report, len)
                #         stat = _('Unknown status')

                # s = wxString::Format(wxT('IP %s %s'), m_ri->m_radar_address.FormatNetworkAddress(), stat.c_str())
                # info = m_ri->GetRadarLocationInfo()
                # s << wxT('\n') << _('SKU ') << MOD_serial << _(' Serial #') << info.serialNr
                # SetInfoStatus(s)
            case 0x00010006:
                pass
                # IF_serial = wxString::FromAscii(data + 4, 7)
                # MOD_serial = wxString::FromAscii(data + 20, 7)
                # m_info = m_ri->GetRadarLocationInfo()
                # m_ri->m_radar_location_info.serialNr = IF_serial
            case 0x00018801:  # HD radar
                pass
                # ProcessRMReport(data, len)
            case 0x00010007:
                pass
            case 0x00010008:
                pass
            case 0x00010009:
                pass
            case 0x00018942:
                # self.get_logger().debug('other frame')
                pass
            case _:
                # self.get_logger().debug('default frame')
                pass


    def process_rm_report(self, data: bytes):
        if len(data) < 186: return # ensure packet is longer than 186 bytes
        
        bl = RMReport.parse_report(data[:260])
        rmr = RMReport(*bl)
        self.get_logger().debug(f'{rmr}')


    def process_quantum_scan_data(self, data: bytes):
        if len(data) < 20: return # ensure packet is longer than 20 bytes
        
        qheader = QuantumScan.parse_header(data[:20])
        qdata = QuantumScan.parse_data(data[20:])
        qs = QuantumScan(*qheader, qdata)
        self.get_logger().debug(f'{qs}')
        
        if self.spokes is None: return

        self.spokes[qs.azimuth, :len(qs.data)] = qs.data
        self.spokes_updated += 1
        self.get_logger().debug(f'Received spoke #{qs.azimuth} ({self.spokes_updated}/{MAX_SPOKE_COUNT})')
                
        # msg = Spoke()
        # msg.azimuth = qs.azimuth
        # msg.data = qs.data
        # self.spoke_publisher.publish(msg)
        # self.get_logger().debug(f'published {msg=}')


    def process_quantum_report(self, data: bytes):
        if len(data) < 260: return # ensure packet is longer than 260 bytes
        
        bl = QuantumReport.parse_report(data[:260])
        qr = QuantumReport(*bl)
        self.get_logger().debug(f'{qr}')
        
        self.last_quantum_report = qr


    def imager_callback(self):
        polar_image = self.get_polar_image()
        self.get_logger().info(f'Updated {self.spokes_updated} spokes in polar image of dimension {polar_image.shape=}')
        
        self.generate_map(r=polar_image, k=None, p=None, K=None, area_threshold=50, gamma=None)
        self.spokes_updated = 0
        


    def get_polar_image(self):
        # Uncomment below for testing
        # self.spokes = np.random.uniform(low=0, high=MAX_INTENSITY, size=(250, MAX_SPOKE_LENGTH)) # random spokes
        # polar_image = np.copy(self.spokes/MAX_INTENSITY * 255).astype(np.uint8)
        polar_image = np.copy(self.spokes/MAX_INTENSITY * 255).astype(np.uint8)
        
        # cv2.imshow('polar image', spokes); cv2.waitKey(0)
        
        return polar_image


    def generate_map(self, r, k, p, K, area_threshold, gamma):
        """
        Args:
            r (_type_): raw radar data
            k (_type_): spline curve degree
            p (_type_): knot spacing
            K (_type_): coordinate transformation matrix
            area_threshold (_type_): minimum polygon area threshold
            gamma (_type_): angular resolution to discretize the polar coordinate 
        """
        I = self.generate_radar_image(r, K)
        # I = cv2.imread('/home/ws/test.png', cv2.IMREAD_GRAYSCALE)
        # I = cv2.resize(I, None, fx=0.5, fy=0.5)
        D = self.detect_contour(self.filter_image(I,binary_threshold=128))
        P = self.extract_coastline(D, area_threshold=10, angular_resolution=None, K=None)

        _, binary_image = cv2.threshold(I, 0, 255, cv2.THRESH_BINARY)
        
        self.image_publisher_.publish(self.bridge.cv2_to_imgmsg(I, encoding="passthrough"))
        # self.image_publisher_.publish(self.bridge.cv2_to_imgmsg(binary_image, encoding="passthrough"))
        # self.image_publisher_.publish(self.bridge.cv2_to_imgmsg(P, encoding="passthrough"))
        self.get_logger().info(f'Published coastline image of dimension {P.shape}')


    def generate_radar_image(self, spokes, K):
        """convert 2d polar image of radar data to 2d cartesian imagecv2.im

        Args:
            spokes (_type_): 2D array of dimension (MAX_SPOKE_COUNT, MAX_SPOKE_LENGTH)
                                 with each row representing a single radar spoke

        Returns:
            radar_image (_type_): grayscale image of radar scan in cartesian coordinates
        """
        radar_image = cv2.warpPolar(
            src=spokes, 
            dsize=(2*MAX_SPOKE_LENGTH, 2*MAX_SPOKE_LENGTH), 
            center=(MAX_SPOKE_LENGTH, MAX_SPOKE_LENGTH), 
            maxRadius=MAX_SPOKE_LENGTH, flags=cv2.WARP_INVERSE_MAP)
        radar_image = cv2.rotate(radar_image, cv2.ROTATE_90_CLOCKWISE)
        
        # self.get_logger().info(f'{radar_image=} {radar_image.shape}')
        # cv2.imshow('raw image', radar_image); cv2.waitKey(0)
        
        return radar_image


    def filter_image(self, radar_image, binary_threshold):
        """apply morphological and bilateral filters for denoising
        and convert grayscale radar_image to binary image according to 
        predetermined threshold intensity value
        
        Args:
            radar_image (_type_): grayscale image of radar scan in cartesian coordinates

        Returns:
            _type_: _description_
        """
        
        # [TODO]: apply morphological and bilateral filters
        erosion = cv2.erode(radar_image, np.ones((3, 3), np.uint8), iterations=1)
        # cv2.imshow('erosion', erosion); cv2.waitKey(0)
        dilation = cv2.dilate(erosion, np.ones((3, 3), np.uint8), iterations=1)
        # cv2.imshow('dilation', dilation); cv2.waitKey(0)
        
        bilateral = cv2.bilateralFilter(dilation, 9, 100, 100)
        # cv2.imshow('bilateral', bilateral); cv2.waitKey(0)
        
        # [TODO]: adjust threshold intensity value. range:[0,255]
        _, binary_image = cv2.threshold(bilateral, binary_threshold, 255, cv2.THRESH_BINARY)
        # cv2.imshow('threshold', binary_image); cv2.waitKey(0)
        
        return binary_image


    def detect_contour(self, binary_image):
        """extract the contours from the binary image using polygon extraction

        Args:
            binary_image (_type_): _description_

        Returns:
            _type_: _description_
        """
        # [TODO]: extract the contours from the binary image using polygon extraction
        
        contours, hierarchy = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return contours


    def extract_coastline(self, contours, area_threshold, angular_resolution=None, K=None):
        landmasses = [cnt for cnt in contours if cv2.contourArea(cnt) > area_threshold]
        
        img = np.zeros((2*MAX_SPOKE_LENGTH, 2*MAX_SPOKE_LENGTH, 3), dtype=np.uint8)
        cv2.drawContours(img, landmasses, -1, (255,255,255), -1)
        # cv2.imshow('contour', img); cv2.waitKey(0)
        
        height, width = img.shape[:2]
        polar = cv2.warpPolar(src=img, dsize=(MAX_SPOKE_LENGTH, 0), center=(width/2, height/2), maxRadius=width/2, flags=cv2.WARP_POLAR_LINEAR)
        polar = cv2.cvtColor(polar, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('polar', polar); cv2.waitKey(0)
        
        for spoke in polar:
            spoke[np.argmax(spoke > 0)+1:] = 0
        
        # cv2.imshow('coastline polar', polar); cv2.waitKey(0)
        
        coastline = cv2.warpPolar(
            src=polar, 
            dsize=(2*MAX_SPOKE_LENGTH, 2*MAX_SPOKE_LENGTH), 
            center=(MAX_SPOKE_LENGTH, MAX_SPOKE_LENGTH), 
            maxRadius=MAX_SPOKE_LENGTH, flags=cv2.WARP_INVERSE_MAP)
        
        # cv2.imshow('coastline', coastline); cv2.waitKey(0)
        
        return coastline


    def radar_control_callback(self, msg):
        match msg.data:
            case 'start_scan':
                self.start_scan()
            case 'stop_scan':
                self.stop_scan()
            case _:
                self.stop_scan()


def main(args=None):
    rclpy.init(args=args)
    quantum = Qauntum()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(quantum)
    executor.spin()
    quantum.destroy_node()
    executor.shutdown()


if __name__ == '__main__':
    main()
