# ping pong server
import socket

def main():
    message = b'pong'
    
    HOST = ''                 # Symbolic name meaning all available interfaces
    PORT = 50007              # Arbitrary non-privileged port
    with socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP) as s:
        s.bind((HOST, PORT))
        print(f'Starting server at {socket.gethostbyname(socket.gethostname())}:{PORT}')
        while True:
            print(f'Waiting for connection')
            data, client_addr = s.recvfrom(1024) # buffer size is 1024 bytes
            print(f'Received {data} from {client_addr}')
            s.sendto(message, client_addr)
            print(f'Sent {message} to {client_addr}')


if __name__ == '__main__':
    main()
