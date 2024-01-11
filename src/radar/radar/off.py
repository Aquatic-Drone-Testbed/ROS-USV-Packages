# ping pong client
import socket
import struct
import time

HOST = '192.168.1.117'        # The remote host
PORT = 2575                 # The same port as used by the server
RADAR_ADDR = (HOST, PORT)

rd_msg_tx_on = bytes([0x10, 0x00, 0x28, 0x00,
                      0x01,  # Control value at offset 4 : 0 - off, 1 - on
                      0x00, 0x00, 0x00])

rd_msg_tx_off = bytes([0x10, 0x00, 0x28, 0x00,
                       0x00,  # Control value at offset 4 : 0 - off, 1 - on
                       0x00, 0x00, 0x00])

# Quantum 1 sec stay alive
stay_alive_1sec = bytes([0x00, 0x00, 0x28, 0x00, 0x52, 0x61,
                         0x64, 0x61, 0x72, 0x00, 0x00, 0x00])

# Quantum stay alive 5 sec message, 36 bytes
rd_msg_5s = bytes([0x03, 0x89, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                   0x9e, 0x03, 0x00, 0x00, 0xb4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

rd_msg_set_range = bytes([0x01, 0x01, 0x28, 0x00, 0x00,
                          0x0f,  # Quantum range index at pos 5
                          0x00, 0x00])

def transmit_command(s, command):
    s.sendto(command, RADAR_ADDR)
    print(f'Sent {len(command)} bytes to {RADAR_ADDR}')

def main():
    with socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP) as m_comm_socket:
        m_comm_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        transmit_command(m_comm_socket, rd_msg_tx_off)



if __name__ == '__main__':
    main()
