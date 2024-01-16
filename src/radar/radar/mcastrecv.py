# Example multicast receiver script

import socket
import struct
from collections import namedtuple
import logging

LocationInfoBlock = namedtuple('LocationInfoBlock', 
                               'field1 field2 model_id field3 field4 field5 field6 data_ip data_port radar_ip radar_port')

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
        
        # Receive/respond loop
        while True:
            # print('waiting to receive message')
            data, senderaddr = sock.recvfrom(1024)
            logging.debug(f'received {len(data)} bytes from {senderaddr}')
            logging.debug(data)
            if len(data) != 36:
                continue
            
            rRec = LocationInfoBlock._make(struct.unpack('!IIBBHIIIIII', data))
            if rRec.model_id != 40: continue
            
            logging.info('RaymarineLocate received RadarReport')
            data_ip = struct.unpack('4B', struct.pack('I', socket.ntohl(rRec.data_ip)))
            data_port = struct.unpack('2H', struct.pack('I', socket.ntohl(rRec.data_port)))
            
            radar_ip = struct.unpack('4B', struct.pack('I', socket.ntohl(rRec.radar_ip)))
            radar_port = struct.unpack('2H', struct.pack('I', socket.ntohl(rRec.radar_port)))
            
            logging.debug(rRec)
            logging.debug(socket.ntohl(rRec.radar_port))
            logging.debug(f'data_addr = {data_ip}:{data_port}')
            logging.debug(f'radar_addr = {radar_ip}:{radar_port}')

            # print(f'sending acknowledgement to {senderaddr}')
            # sock.sendto(bytearray("ack", "utf-8"), senderaddr)


if __name__ == '__main__':
    main()
