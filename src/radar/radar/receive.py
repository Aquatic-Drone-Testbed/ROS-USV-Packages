# Example multicast receiver script

import socket
import struct
import logging

logging.basicConfig(level=logging.INFO)

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
            logging.debug(data)


if __name__ == '__main__':
    main()
