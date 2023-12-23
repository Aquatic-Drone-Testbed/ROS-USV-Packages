# server.py
import socket

def main():
    # Create a socket object
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Get local machine name
    host = socket.gethostname()

    # Port for your service
    port = 20000

    # Connection to hostname on the port
    client_socket.connect((host, port))

    # Send a message
    client_socket.send("Hello World".encode('utf-8'))

    # Close the socket
    client_socket.close()


if __name__ == '__main__':
    main()
