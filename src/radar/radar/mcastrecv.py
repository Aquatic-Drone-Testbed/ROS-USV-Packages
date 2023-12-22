# Example multicast receiver script

import socket
import struct

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
            print('waiting to receive message')
            data, senderaddr = sock.recvfrom(1024)

            print(f'received {len(data)} bytes from {senderaddr}')
            print(data)

            print(f'sending acknowledgement to {senderaddr}')
            sock.sendto(bytearray("ack", "utf-8"), senderaddr)


if __name__ == '__main__':
    main()
