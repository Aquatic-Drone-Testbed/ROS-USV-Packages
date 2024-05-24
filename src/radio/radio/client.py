import socket
from time import sleep

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

ctrl_station_ip = '10.223.75.168'
server_address = (ctrl_station_ip, 9000)

try:
    while True:
        message = 'Ping'
        print(f"Sending: {message}")
        sock.sendto(message.encode(), server_address)

        data, server = sock.recvfrom(4096)
        if data:
            print(f"Received response: {data.decode()}")

        sleep(1)

finally:
    print("Closing socket")
    sock.close()
