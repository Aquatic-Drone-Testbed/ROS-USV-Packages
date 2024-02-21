import socket
import radar.locate
import radar.control
import radar.receive
import struct
from threading import Thread
import logging

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

SCAN_DURATION_SECONDS = 10

def main():
    radar_ip, radar_port, data_ip, data_port = radar.locate.detect_radar()
    
    # Create command socket
    with socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP) as command_socket:
        command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        command_socket.setblocking(0) # non-blocking socket
        logger.info(f'Initialized command_socket')
        
        logger.info(f'Run radar keep alive thread')
        keep_alive_thread = Thread(target = radar.control.standby_radar, args=(command_socket, radar_ip, radar_port))
        keep_alive_thread.start()
        
        radar.control.radar_tx_on(command_socket=command_socket,
                                  command_ip=radar_ip,
                                  command_port=radar_port)
        
        # Create report socket
        with socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP) as report_socket:
            report_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            # Bind to the server address
            # on this port, receives ALL multicast groups
            report_socket.bind(('', data_port))
            
            # Tell the operating system to add the socket to the multicast group
            # on all interfaces.
            mreq = struct.pack('4sL', socket.inet_aton(data_ip), socket.INADDR_ANY)
            report_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            logger.info(f'Initialized report_socket')
            
            logger.info(f'Run radar receive thread')
            receive_thread = Thread(target = radar.receive.listen_radar, 
                                    args=(report_socket,))
            receive_thread.start()
            receive_thread.join(SCAN_DURATION_SECONDS)
            logger.info(f'Ending receive thread')
        
        radar.control.radar_tx_off(command_socket=command_socket,
                                   command_ip=radar_ip,
                                   command_port=radar_port)
        keep_alive_thread.join(1)

if __name__ == '__main__':
    main()
