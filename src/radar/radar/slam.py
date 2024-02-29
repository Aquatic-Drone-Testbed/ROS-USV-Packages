import struct

from radar_interfaces.srv import RadarData
import rclpy
from rclpy.node import Node

from radar.packets.rmr_report import RMReport
from radar.packets.quantum_scan import QuantumScan
from radar.packets.quantum_report import QuantumReport

class Slam(Node):

    def __init__(self):
        super().__init__('slam')
        self.cli = self.create_client(RadarData, 'radar_data')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')


    def reqest_radar_data(self):
        self.future = self.cli.call_async(RadarData.Request())
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


    def process_frame(self, data: bytes):
        self.get_logger().info(f'Processing {len(data)} bytes')
        # self.get_logger().debug(f'{data}')
        if len(data) < 4: return # data must be longer than 4 bytes
        
        msg_id = struct.unpack('<I', data[:4])[0] # read first 4 bytes
        
        match msg_id:
            case 0x00010001:
                self.process_rm_report(data)
                pass
            case 0x00010002:
                # ProcessFixedReport(data, len)
                pass
            case 0x00010003:
                # ProcessScanData(data, len)
                pass
            case 0x00280003:
                self.process_quantum_scan_data(data)
                pass
            case 0x00280002:
                self.process_quantum_report(data)
                pass
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


    def process_quantum_report(self, data: bytes):
        if len(data) < 260: return # ensure packet is longer than 260 bytes
        
        bl = QuantumReport.parse_report(data[:260])
        qr = QuantumReport(*bl)
        self.get_logger().debug(f'{qr}')


def main():
    rclpy.init()

    slam = Slam()
    while True:
        response = slam.reqest_radar_data()
        slam.process_frame(response.data)

    slam.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
