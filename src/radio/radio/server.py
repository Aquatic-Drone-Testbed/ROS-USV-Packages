import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# get local machine name
host = socket.gethostname()
print(f"I am {host}")

port = 30000

server_address = ('', port)
sock.bind(server_address)

print(f"Listening for messages on {port}...")

while True:
    data, address = sock.recvfrom(4096)

    if data:
        message = data.decode()
        print(f"Received message: {message} from {address}")

        # Respond to client
        response = 'Pong'
        print(f"Sending response: {response} to {address}")
        sock.sendto(response.encode(), address)
