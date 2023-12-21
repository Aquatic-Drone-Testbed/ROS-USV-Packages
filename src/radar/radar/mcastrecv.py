# Example multicast receiver script

import socket
import struct

def main():
    multicast_group = '224.0.0.1'
    # multicast_group = '232.1.1.1'
    multicast_port = 5800

    # Create the socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Bind to the server address
    sock.bind((multicast_group, multicast_port))
    
    # Tell the operating system to add the socket to the multicast group
    # on all interfaces.
    mreq = struct.pack('4sL', socket.inet_aton(multicast_group), socket.INADDR_ANY)
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
