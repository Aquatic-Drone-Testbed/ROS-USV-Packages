import socket
import struct
import select

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from radar_interfaces.msg import Spoke

from radar.packets.location_info import LocationInfo
from radar.packets import control_message
from radar.packets.rmr_report import RMReport
from radar.packets.quantum_scan import QuantumScan
from radar.packets.quantum_report import QuantumReport

KM_PER_NMI = 1.852

class FrameId:
    RM_REPORT = 0x00010001
    QUANTUM_REPORT = 0x00280002
    QUANTUM_SPOKE = 0x00280003


class Qauntum(Node):

    def __init__(self):
        super().__init__('quantum')
        # uncomment below to simulate locator
        data = bytes.fromhex('00000000928b80cb28000000030064000608100001b301e80e0a11007501a8c00f0a3600')
        bl = LocationInfo.parse(data[:36])
        self.quantum_location = LocationInfo(*bl)
        # self.quantum_location = self.locate_quantum() # this might take a while to return
        
        self.command_socket = self.create_command_socket() # socket for controlling radar
        self.report_socket = self.create_report_socket() # socket for receiving radar data
        
        self.last_frame_id = 0
        self.last_quantum_report = None
        self.last_quantum_spoke = None
        
        # timer
        self.get_logger().info(f'Keeping radar alive')
        self.standby_timer = self.create_timer(1, self.standby_timer_callback) # 1 second timer
        self.alive_counter = 0
        
        # publisher
        self.radar_spoke_publisher = self.create_publisher(
            Spoke, 
            'topic_radar_spoke', 
            10)
        
        # subscription
        self.radar_control_subscription = self.create_subscription(
            String,
            'topic_radar_control',
            self.radar_control_callback,
            10)


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
            self.get_logger().debug(f'received ({len(data)} bytes) from {senderaddr}')
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


    def create_report_socket(self) -> socket:
        report_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP)
        report_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Bind to the server address
        # on this port, receives ALL multicast groups
        report_socket.bind(('', self.quantum_location.data_port))
        # Tell the operating system to add the socket to the multicast group
        # on all interfaces.
        mreq = struct.pack('4sL', socket.inet_aton(self.quantum_location.data_ip), socket.INADDR_ANY)
        report_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        
        report_socket.setblocking(0) # non-blocking socket
        
        self.get_logger().info(f'Initialized report socket')
        
        return report_socket


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
        self.get_logger().debug(f'Sent ({len(command)} bytes) to {self.quantum_location.radar_ip,}:{self.quantum_location.radar_port}')


    def radar_tx_on(self) -> None:
        self.transmit_command(control_message.TX_ON)
        self.get_logger().info(f'Sent tx on command')


    def radar_tx_off(self) -> None:
        self.transmit_command(control_message.TX_OFF)
        self.get_logger().info(f'Sent tx off command')


    def radar_control_callback(self, msg):
        match msg.data:
            case 'start_scan':
                self.radar_tx_on()
            case 'stop_scan':
                self.radar_tx_off()
            case _:
                self.radar_tx_off()


    def get_radar_data(self, timeout_sec=1):
        try:
            read_sockets, write_sockets, error_sockets = select.select([self.report_socket], [], [], timeout_sec)
            if not read_sockets: return
            
            data, senderaddr = self.report_socket.recvfrom(1024)
        except OSError:
            return
        
        return data


    def process_frame(self, data: bytes):
        if (data is None) or (len(data) < 4): return # data must be longer than 4 bytes
        
        self.last_frame_id = struct.unpack('<I', data[:4])[0] # read first 4 bytes
        self.get_logger().debug(f'{self.last_frame_id=} ({len(data)} bytes)')
        # self.get_logger().debug(f'{data}')
        
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
        
        self.last_quantum_spoke = qs
        
        msg = Spoke()
        msg.azimuth = qs.azimuth
        msg.data = qs.data
        self.radar_spoke_publisher.publish(msg)
        self.get_logger().debug(f'published {msg=}')


    def process_quantum_report(self, data: bytes):
        if len(data) < 260: return # ensure packet is longer than 260 bytes
        
        bl = QuantumReport.parse_report(data[:260])
        qr = QuantumReport(*bl)
        self.get_logger().debug(f'{qr}')
        
        self.last_quantum_report = qr


def main(args=None):
    rclpy.init(args=args)

    quantum = Qauntum()
    while True:
        rclpy.spin_once(quantum)
        quantum.process_frame(quantum.get_radar_data(timeout_sec=0.1))
    quantum.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
