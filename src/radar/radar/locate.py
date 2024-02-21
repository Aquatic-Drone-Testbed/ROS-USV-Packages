# Listen on multicast addr 224.0.0.1:5800 to find radar's location on the network

import socket
import struct
from collections import namedtuple
import logging

logger = logging.getLogger(__name__)

LocationInfoBlock = namedtuple('LocationInfoBlock', 
                               ['field1', # u32
                                'field2', # u32
                                'model_id', # u8
                                'field3', # u16
                                'field4', # u32
                                'field5', # u32
                                'field6', # u32
                                'data_ip', # u32
                                'data_port', # u32
                                'radar_ip', # u32
                                'radar_port']) # u32

def detect_radar():
    MULTICAST_GROUP = '224.0.0.1'
    MULTICAST_PORT = 5800
    QUANTUM_MODEL_ID = 40
    
    # Create the socket
    with socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP) as locator_socket:
        locator_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Bind to the server address
        # on this port, receives ALL multicast groups
        locator_socket.bind(('', MULTICAST_PORT))
        
        # Tell the operating system to add the socket to the multicast group
        # on all interfaces.
        mreq = struct.pack('4sL', socket.inet_aton(MULTICAST_GROUP), socket.INADDR_ANY)
        locator_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        logger.info(f'Initialized locator socket')
        
        # Receive/respond loop
        while True:
            logger.info(f'locating radar...')
            data, senderaddr = locator_socket.recvfrom(1024)
            logger.debug(f'received ({len(data)} bytes) from {senderaddr}')
            if len(data) != 36:
                continue
            
            location_info = LocationInfoBlock._make(struct.unpack('IIBBHIIIIII', data))
            if location_info.model_id != QUANTUM_MODEL_ID:
                # logger.warning(f'{location_info.model_id=} is not quantum')
                continue
            
            data_ip_tup = struct.unpack('4B', struct.pack('I', socket.ntohl(location_info.data_ip)))
            data_port_tup = struct.unpack('2H', struct.pack('>I', socket.ntohl(location_info.data_port)))
            data_ip = '.'.join([str(x) for x in data_ip_tup])
            data_port = data_port_tup[0]
            
            radar_ip_tup = struct.unpack('4B', struct.pack('I', socket.ntohl(location_info.radar_ip)))
            radar_port_tup = struct.unpack('2H', struct.pack('>I', socket.ntohl(location_info.radar_port)))
            radar_ip = '.'.join([str(x) for x in radar_ip_tup])
            radar_port = radar_port_tup[0]
            
            logger.info(f'Found radar at {radar_ip}')
            
            return radar_ip, radar_port, data_ip, data_port
