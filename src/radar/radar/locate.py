# Listen on multicast addr 224.0.0.1:5800 to find radar's location on the network

import socket
import struct
from radar.packets.LocationInfo import LocationInfo
import logging

logger = logging.getLogger(__name__)

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
            
            li = LocationInfo(*LocationInfo.parse(data))
            logger.debug(f'{li=}')
            if li.model_id != QUANTUM_MODEL_ID:
                # logger.warning(f'{location_info.model_id=} is not quantum')
                continue
            
            logger.info(f'Found radar at {li.radar_ip}')
            
            return li
