# ping pong client
import socket

def main():
    rd_msg_tx_control = bytes([0x10, 0x00, 0x28, 0x00,
                               0x01,  # Control value at offset 4 : 0 - off, 1 - on
                               0x00, 0x00, 0x00])
    message = b'ping'
    # message = rd_msg_tx_control
    
    HOST = '192.168.1.91'        # The remote host
    PORT = 50007              # The same port as used by the server
    SERVER = (HOST, PORT)
    with socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP) as s:
        s.sendto(message, SERVER)
        print(f'Sent {message} to {SERVER}')
        data, server_addr = s.recvfrom(1024)
        print(f'Received {data} from {server_addr}')


if __name__ == '__main__':
    main()
