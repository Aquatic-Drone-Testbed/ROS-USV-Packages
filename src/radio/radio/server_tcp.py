# server.py
import socket

def main():
    # Create a socket object
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Set IP address and port
    host = '0.0.0.0'  # Listen on all available interfaces
    port = 20000

    # Bind to the port
    server_socket.bind((host, port))

    # Queue up to 5 requests
    server_socket.listen(5)

    print(f"Server listening on {host}:{port}")

    while True:
        # Establish a connection
        client_socket, addr = server_socket.accept()
        print("Got a connection from %s" % str(addr))
        print(addr)
        # Receive and print data (1024 bytes)
        data = client_socket.recv(1024)
        print("Received: %s" % data.decode('utf-8'))
        
        # Close the connection
        client_socket.close()


if __name__ == '__main__':
    main()
