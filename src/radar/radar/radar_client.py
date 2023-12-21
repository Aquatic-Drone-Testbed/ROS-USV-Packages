# Echo client program
import socket

def main():
    rd_msg_tx_control = bytes([0x10, 0x00, 0x28, 0x00,
                               0x01,  # Control value at offset 4 : 0 - off, 1 - on
                               0x00, 0x00, 0x00])
    
    HOST = 'localhost'        # The remote host
    PORT = 50007              # The same port as used by the server
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        s.sendall(rd_msg_tx_control)
        data = s.recv(2048)
    print('Received', repr(data))


if __name__ == '__main__':
    main()
