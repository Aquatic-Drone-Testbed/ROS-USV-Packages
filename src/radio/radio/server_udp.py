# server.py
import socket

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    server_address = ('10.223.75.168', 20000) 

    try:
        message = 'Hello World'
        print(f"Sending: {message}")
        sent = sock.sendto(message.encode(), server_address)

    finally:
        print("Closing socket")
        sock.close()


if __name__ == '__main__':
    main()
