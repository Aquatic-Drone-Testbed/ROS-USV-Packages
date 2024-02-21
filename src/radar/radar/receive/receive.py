# Listen on multicast addr 232.1.179.1:2574 for radar data
import socket
import struct
import logging

import radar.receive.process_report
from radar.receive.QuantumScan import QuantumScan
from radar.receive.QuantumReport import QuantumReport

from collections import namedtuple

logger = logging.getLogger(__name__)

def listen_radar(report_socket: socket):
    while True:
        try:
            data, senderaddr = report_socket.recvfrom(1024)
        except OSError:
            break
        # logger.debug(f'received ({len(data)} bytes) from {senderaddr}')
        
        process_frame(data)


def process_frame(data: bytes):
    if len(data) < 4: # data must be longer than 4 bytes
        return
    
    msg_id = struct.unpack('<I', data[:4])[0] # read first 4 bytes
    # logger.debug(f'received frame with {hex(msg_id)=}')
    
    match msg_id:
        case 0x00010001:
            # radar.receive.process_report.ProcessRMReport(data)
            pass
        case 0x00010002:
            # ProcessFixedReport(data, len)
            pass
        case 0x00010003:
            # ProcessScanData(data, len)
            pass
        case 0x00280003:
            process_quantum_scan_data(data)
            pass
        case 0x00280002:
            process_quantum_report(data)
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
            # logger.debug('other frame')
            pass
        case _:
            # logger.debug('default frame')
            pass


def process_quantum_scan_data(data: bytes):
    if len(data) < 20: # ensure packet is longer than 20 bytes
        return
    
    qheader = QuantumScan.parse_header(data[:20])
    qdata = QuantumScan.parse_data(data[20:])
    qs = QuantumScan(*qheader, qdata)
    logger.debug(f'{qs}')


def process_quantum_report(data: bytes):
    if len(data) < 260: # ensure packet is longer than 260 bytes
        return
    
    bl = QuantumReport.parse_report(data[:260])
    qr = QuantumReport(*bl)
    logger.debug(f'{qr=}')
