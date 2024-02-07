# Listen on multicast addr 232.1.179.1:2574 for radar data
import socket
import struct
import logging

logging.basicConfig(level=logging.DEBUG)

def main():
    MULTICAST_GROUP = '232.1.179.1'
    MULTICAST_PORT = 2574

    # Create the socket
    with socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Bind to the server address
        # on this port, receives ALL multicast groups
        sock.bind(('', MULTICAST_PORT))
        
        # Tell the operating system to add the socket to the multicast group
        # on all interfaces.
        mreq = struct.pack('4sL', socket.inet_aton(MULTICAST_GROUP), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        logging.info(f'initialized {sock=}')
        
        # Receive/respond loop
        while True:
            # print('waiting to receive message')
            data, senderaddr = sock.recvfrom(1024)
            logging.debug(f'received {len(data)} bytes from {senderaddr}')
            # logging.debug(data)
            
            process_frame(data)

def process_frame(data: bytes):
    if len(data) < 4: # data must be longer than 4 bytes
        return
    
    msg_id = struct.unpack('!I', data[:4]) # read first 4 bytes
    logging.debug(f'received frame with {msg_id=}')
    
    match msg_id:
        case 0x00010001:
            # ProcessRMReport(data, len)
            pass
        case 0x00010002:
            # ProcessFixedReport(data, len)
            pass
        case 0x00010003:
            # ProcessScanData(data, len)
            pass
        case 0x00280003:
            # ProcessQuantumScanData(data, len)
            pass
        case 0x00280002:
            # ProcessQuantumReport(data, len)            
            pass
        case 0x00280001:  # type and serial for Quantum radar
            logging.debug('received frame 0x00280001')
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
            logging.debug('received frame 0x00010006')
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
            logging.debug('received frame other')
        case _:
            logging.debug('received frame default')

if __name__ == '__main__':
    main()
