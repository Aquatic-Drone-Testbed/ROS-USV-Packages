import socket
import struct
import numpy as np
import cv2
from cv_bridge import CvBridge
import time
import os

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
import radar.filter as RadarFilter

#testing values
PREV_SPOKE = 0

MAX_SPOKE_LENGTH = 256
DEFAULT_NUM_SPOKES = 250
MAX_INTENSITY = 128
MIN_RANGE_INDEX = 0
MAX_RANGE_INDEX = 20

class FrameId:
    RM_REPORT = 0x00010001
    QUANTUM_REPORT = 0x00280002
    QUANTUM_SPOKE = 0x00280003

class Qauntum(Node):

    def __init__(self):
        super().__init__('quantum_node')
        
        self.declare_parameter('host', "127.0.0.1")  # Default ip addr is local host
        self.host = self.get_parameter('host').get_parameter_value().string_value
        
        # uncomment below to simulate locator
        # data = b'\x00\x00\x00\x00\x92\x8b\x80\xcb(\x00\x00\x00\x03\x00d\x00\x06\x08\x10\x00\x01\xb3\x01\xe8\x0e\n\x11\x002\x00\xe0\n\x0f\n6\x00'
        # bl = LocationInfo.parse(data[:36])
        # self.quantum_location = LocationInfo(*bl)
        self.quantum_location = self.locate_quantum() # this might take a while to return
        
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
        # request polar image every 2.5 second (2.65 to compensate for lag)
        self.polar_image_timer = self.create_timer(
            2.55, 
            self.imager_callback, 
            reentrant_callback_group)
        
        # request full scan in string form every 2.5 seconds
        # self.scan_string_timer = self.create_timer(
        #     2.55, 
        #     self.scan_string_callback, 
        #     reentrant_callback_group)

        # request data from radar asap
        self.data_timer = self.create_timer(
            0, 
            self.radar_data_callback, 
            receiver_mutex_callback_group)
        # send radar heartbeat every 3 second
        self.diagnostic_timer = self.create_timer(
            3.0,
            self.publish_radar_heartbeat)
        
        # publisher
        self.image_publisher_ = self.create_publisher(
            Image, 
            'radar_image', 
            10, 
            callback_group=reentrant_callback_group)
        self.polar_image_publisher = self.create_publisher(
            Image, 
            '/USV/Polar', 
            10, 
            callback_group=reentrant_callback_group)

        self.spoke_pub = self.create_publisher(
            String, 
            'radar_spoke', 
            10,
            callback_group=reentrant_callback_group)
        
        self.scan_string_pub = self.create_publisher(
            String, 
            'radar_scan_str', 
            10,
            callback_group=reentrant_callback_group)

        self.diagnostic_pub = self.create_publisher(
            String, 
            'diagnostic_status', 
            10)
        
        # subscription
        self.radar_control_subscription = self.create_subscription(
            String,
            'radar_control',
            self.radar_control_callback,
            10,
            callback_group=command_mutex_callback_group_)
        
        self.imu_subscription = self.create_subscription(
            String,
            'imu_handler_data',
            self.process_imu_input,
            10)
    
        self.alive_counter = 0
        self.num_spokes = DEFAULT_NUM_SPOKES
        self.spokes = np.zeros((self.num_spokes, MAX_SPOKE_LENGTH), np.uint8)
        self.spokes_updated = 0
        self.set_zoom(0)
        self.bridge = CvBridge()
        self.scanning = False

        self.imu_input = ''

        self.scan_str = ''

        ### TIMER FOR IMU ONLY, COMMENT OUT WHEN IN NORMAL OPERATION

        # self.imu_timer = self.create_timer(
        #     0.01, 
        #     self.send_imu_only, 
        #     reentrant_callback_group)

    def locate_quantum(self, multicast_group='224.0.0.1', multicast_port=5800, quantum_model_id=40) -> LocationInfo:
        # Create UDP socket
        locator_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP)
        locator_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Bind to the server address
        # on this port, receives ALL multicast groups
        locator_socket.bind(('', multicast_port))
        
        # Tell the operating system to add the socket to the multicast group
        # on all interfaces.
        
        # Bind socket to proper network interface
        #locator_socket.setsockopt(socket.SOL_SOCKET, 25, 'enxa0cec8b67b9f')
        
        
        mreq = struct.pack('4sL', socket.inet_aton(multicast_group), socket.INADDR_ANY)
        locator_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        self.get_logger().info(f'Initialized locator socket')
        
        # loop until Raymarine Quantum is found #NOTE: IP IS PROBABLY 10.42.0.253, PORT IS PROBABLY 2575
        self.get_logger().info(f'Locating radar...')
        # while True:
        #     data, senderaddr = locator_socket.recvfrom(1024)
        #     self.get_logger().debug(f'received {len(data)} bytes from {senderaddr[0]}:{senderaddr[1]}')
        #     if len(data) != 36: continue # ignore any packets not 36 bytes
            
        #     quantum_location = LocationInfo(*LocationInfo.parse(data))
        #     self.get_logger().debug(f'{quantum_location=}')
        #     print(f'{quantum_location=}')
        #     if quantum_location.model_id != quantum_model_id: continue
            
        #     self.get_logger().info(f'Found radar at {quantum_location.radar_ip}')

        #     print(quantum_location.radar_port)
        #     break
        
        quantum_location = LocationInfo(field1=0, field2=3414199186, model_id=40, field3=0, field4=0, field5=6553603, field6=1050630, data_ip='232.1.179.1', data_port=2574, radar_ip='10.42.0.253', radar_port=2575)
        self.get_logger().info(f'Found radar at {quantum_location.radar_ip}')

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
        self.scanning = True


    def stop_scan(self) -> None:
        self.transmit_command(control_message.TX_OFF)
        self.get_logger().info(f'Sent tx off command')
        self.scanning = False


    def set_zoom(self, i) -> None:
        self.range_index = i
        self.transmit_command(control_message.get_range_command(i))
        self.get_logger().info(f'Sent zoom level {i} command')


    def zoom_in(self) -> None:
        self.set_zoom(max(self.range_index - 1, MIN_RANGE_INDEX))


    def zoom_out(self) -> None:
        self.set_zoom(min(self.range_index + 1, MAX_RANGE_INDEX))


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
        #print(f'Received {len(data)} bytes (frame_id=0x{self.last_frame_id:X})')
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

    def process_imu_input(self, msg):
        self.imu_input = msg.data

    def process_quantum_scan_data(self, data: bytes): #SENDS RADAR SPOKES, COMMENT OUT THIS FUNCTION FOR IMU TESTING
        
        global PREV_SPOKE
        
        if len(data) < 20: return # ensure packet is longer than 20 bytes
        
        qheader = QuantumScan.parse_header(data[:20])
        qdata = QuantumScan.parse_data(data[20:])
        qs = QuantumScan(*qheader, qdata)
        
        # f = open("/home/ws/test/slam_radar_test/radar_data.txt", "a") 
        # f.write(f'[{time.time()}]\tQ_Header<{qheader}>\tQ_Data<{qdata}>\t{self.imu_input}\n')
        spoke_string = f'[{time.time()}]\tQ_Header<{qheader}>\tQ_Data<{qdata}>\t{self.imu_input}\n'
        self.scan_str += spoke_string

        ##Uncomment below to use spoke string publisher##
        # msg = String()
        # msg.data = spoke_string
        # self.spoke_pub.publish(msg)

        self.get_logger().debug(f'{qs}')
        
        if self.spokes is None: return
        self.num_spokes = qs.num_spokes
        # Quantum Q24C spokes are 180 degrees out of phase
        # i.e. 0th azimuth points towards the **back** of the radar
        # and 125th azimuth points towards the **front** of the radar
        self.spokes[(qs.azimuth + DEFAULT_NUM_SPOKES//2)%DEFAULT_NUM_SPOKES, :len(qs.data)] = qs.data
        self.spokes_updated += 1
        self.get_logger().debug(f'Received spoke #{qs.azimuth} ({self.spokes_updated}/{self.num_spokes})')
        
        #if qs.azimuth != ((PREV_SPOKE+1) % DEFAULT_NUM_SPOKES):
            #print("SPOKE SEQUENCE MISMATCH! " + str(qs.azimuth) + " " + str(PREV_SPOKE) + "\n")
        
        #PREV_SPOKE = qs.azimuth
        
        # msg = Spoke()
        # msg.azimuth = qs.azimuth
        # msg.data = qs.data
        # self.spoke_publisher.publish(msg)
        # self.get_logger().debug(f'published {msg=}')

    # def process_quantum_scan_data(self, data: bytes): # PLACEHOLDER FOR IMU TESTING, COMMENT FOR NORMAL OPERATION
    #     pass  


    # Function that generates a dummy spoke to send live IMU only
    def send_imu_only(self):
        dummy_header = f'(2621443, 15728, 257, 173, 250, 8, 58, 64, 34)'
        dummy_data= f'(0, 0, 0, 80, 61, 76, 35, 77, 0, 77, 0, 0, 0, 0, 0, 0, 14, 58, 91, 49, 55, 53, 11, 15, 41, 0, 21, 28, 0, 0, 36, 0, 0, 16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)'
        dummy_spoke = f'[{time.time()}]\tQ_Header<{dummy_header}>\tQ_Data<{dummy_data}>\t{self.imu_input}\n'
        msg = String()
        msg.data = dummy_spoke
        self.spoke_pub.publish(msg)

    def process_quantum_report(self, data: bytes):
        if len(data) < 260: return # ensure packet is longer than 260 bytes
        
        bl = QuantumReport.parse_report(data[:260])
        qr = QuantumReport(*bl)
        self.range_index = qr.range_index
        self.get_logger().debug(f'{qr}')
        
        self.last_quantum_report = qr


    def imager_callback(self):
        #if self.spokes_updated < 250:
        #    return
        #print(f'imager_callback spokes_updated before: {self.spokes_updated}')
        #polar_image = np.copy(self.spokes/MAX_INTENSITY * 255).astype(np.uint8)
        ### polar_image = np.copy(self.spokes).astype(np.uint8) #
        #cv2.imwrite('test/polar_image.jpg', polar_image)
        #cv2.imwrite(f'test/slam_radar_test/polar/polar_{time.time()}.jpg', polar_image)

        ### self.polar_image_publisher.publish(self.bridge.cv2_to_imgmsg(polar_image, encoding="passthrough")) #
        
        ### I, D, P = RadarFilter.generate_map(r=polar_image, k=None, p=None, K=None, area_threshold=50, gamma=None) #
        
        ###ctrl_station_img = I #
        ###self.image_publisher_.publish(self.bridge.cv2_to_imgmsg(ctrl_station_img, encoding="passthrough")) #
        
        self.get_logger().info(f'Published images (updated {self.spokes_updated} spokes at zoom level {self.range_index})')
        
        self.spokes = np.zeros((self.num_spokes, MAX_SPOKE_LENGTH), np.uint8)
        #self.spokes = np.full((self.num_spokes, 96), 255)
        self.spokes_updated = 0

    def scan_string_callback(self):
        # uncomment below to enable whole scan string publisher
        pass
        #msg = String()
        #msg.data = self.scan_str

        #self.scan_string_pub.publish(msg)
        #self.scan_str = ''

    def radar_control_callback(self, msg):
        match msg.data:
            case 'start_scan':
                self.start_scan()
            case 'stop_scan':
                self.stop_scan()
            case 'toggle_scan':
                self.scanning = not self.scanning
                if self.scanning: self.start_scan()
                else: self.stop_scan()
            case 'zoom_in':
                self.zoom_in()
            case 'zoom_out':
                self.zoom_out()
            case _:
                self.get_logger().error(f'unknown radar control: {msg.data}')


    def publish_radar_heartbeat(self):
        if self.scanning:
            self.diagnostic_pub.publish(String(data="Radar: Scanning"))
        else:
            self.diagnostic_pub.publish(String(data="Radar: Standby"))


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
