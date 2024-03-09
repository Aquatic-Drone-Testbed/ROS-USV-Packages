import socket
import numpy as np
import cv2

# Constants for easy adjustments
IP = '127.0.0.1'
PORT = 9001
BUFFER_SIZE = 65535

def receive_udp_data(ip=IP, port=PORT, buffer_size=BUFFER_SIZE):
    # Use context manager for socket to ensure it's properly closed
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((ip, port))
        print(f"Listening for UDP packets on {ip}:{port}...")

        try:
            while True:
                data, addr = sock.recvfrom(buffer_size)
                print(f"Received packet from {addr}, {len(data)} bytes")

                # Convert the bytes data to a numpy array then decode the image (in color)
                img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)

                if img is not None: # Check that decode was successful
                    cv2.imshow('UDP Stream', img)
                    if cv2.waitKey(1) & 0xFF == ord('q'): # If q is pressed on keyboard
                        break
                else:
                    print("Could not decode image data")
        except KeyboardInterrupt:
            print("\nStopped by user")
        finally:
            cv2.destroyAllWindows()
            print("Socket closed")

if __name__ == "__main__":
    receive_udp_data()
