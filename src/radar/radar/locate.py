# Listen on multicast addr 224.0.0.1:5800 to find radar's location on the network

import socket
import struct
from collections import namedtuple
import logging

logging.basicConfig(level=logging.INFO)

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

def main():
    MULTICAST_GROUP = '224.0.0.1'
    # MULTICAST_GROUP = '232.1.1.1'
    MULTICAST_PORT = 5800

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
            logging.debug(data)
            if len(data) != 36:
                continue
            
            rRec = LocationInfoBlock._make(struct.unpack('IIBBHIIIIII', data))
            if rRec.model_id != 40:
                continue
            
            logging.info('RaymarineLocate received RadarReport')
            data_ip_tup = struct.unpack('4B', struct.pack('I', socket.ntohl(rRec.data_ip)))
            data_ip = '.'.join([str(x) for x in data_ip_tup])
            data_port_tup = struct.unpack('2H', struct.pack('>I', socket.ntohl(rRec.data_port)))
            data_port = data_port_tup[0]
            
            radar_ip_tup = struct.unpack('4B', struct.pack('I', socket.ntohl(rRec.radar_ip)))
            radar_ip = '.'.join([str(x) for x in radar_ip_tup])
            radar_port_tup = struct.unpack('2H', struct.pack('>I', socket.ntohl(rRec.radar_port)))
            radar_port = radar_port_tup[0]
            
            logging.debug(rRec)
            logging.info(f'data_addr = {data_ip}:{data_port}')
            logging.info(f'radar_addr = {radar_ip}:{radar_port}')


if __name__ == '__main__':
    main()
