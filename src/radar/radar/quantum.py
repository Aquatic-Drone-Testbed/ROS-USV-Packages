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
import radar.filter as RadarFilter

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
    
        self.alive_counter = 0
        self.num_spokes = DEFAULT_NUM_SPOKES
        self.spokes = np.zeros((self.num_spokes, MAX_SPOKE_LENGTH), np.uint8)
        self.spokes_updated = 0
        self.set_zoom(0)
        self.bridge = CvBridge()
        self.scanning = False


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
        self.get_logger().info(f'Sent stay alive command')


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

        self.num_spokes = qs.num_spokes
        self.spokes[qs.azimuth, :len(qs.data)] = qs.data
        self.spokes_updated += 1
        self.get_logger().debug(f'Received spoke #{qs.azimuth} ({self.spokes_updated}/{self.num_spokes})')
        
        # msg = Spoke()
        # msg.azimuth = qs.azimuth
        # msg.data = qs.data
        # self.spoke_publisher.publish(msg)
        # self.get_logger().debug(f'published {msg=}')


    def process_quantum_report(self, data: bytes):
        if len(data) < 260: return # ensure packet is longer than 260 bytes
        
        bl = QuantumReport.parse_report(data[:260])
        qr = QuantumReport(*bl)
        self.range_index = qr.range_index
        self.get_logger().debug(f'{qr}')
        
        self.last_quantum_report = qr


    def imager_callback(self):
        polar_image = np.copy(self.spokes/MAX_INTENSITY * 255).astype(np.uint8)
        cv2.imwrite('test/polar_image.jpg', polar_image)
        
        self.polar_image_publisher.publish(self.bridge.cv2_to_imgmsg(polar_image, encoding="passthrough"))
        self.get_logger().info(f'Published polar image of dimension {polar_image.shape} (updated {self.spokes_updated} spokes at zoom level {self.range_index})')
        
        I, D, P = RadarFilter.generate_map(r=polar_image, k=None, p=None, K=None, area_threshold=50, gamma=None)
        
        ctrl_station_img = I
        self.image_publisher_.publish(self.bridge.cv2_to_imgmsg(ctrl_station_img, encoding="passthrough"))
        self.get_logger().info(f'Published image of dimension {ctrl_station_img.shape}')
        
        self.spokes = np.zeros((self.num_spokes, MAX_SPOKE_LENGTH), np.uint8)
        self.spokes_updated = 0


    def radar_control_callback(self, msg):
        match msg.data:
            case 'start_scan':
                self.start_scan()
            case 'stop_scan':
                self.stop_scan()
            case 'zoom_in':
                self.zoom_in()
            case 'zoom_out':
                self.zoom_out()
            case _:
                self.stop_scan()

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
