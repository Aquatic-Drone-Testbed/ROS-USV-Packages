# library for controlling the radar

import socket
import logging

# logging.basicConfig(level=logging.INFO) # Uncomment to enable info logging
logging.basicConfig(level=logging.DEBUG) # Uncomment to enable debug logging

HOST = '192.168.1.117'        # The remote host
PORT = 2575                 # The same port as used by the server
RADAR_ADDR = (HOST, PORT)

TIMEOUT_IN_SECONDS = 1

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

def transmit_command(s: socket, command: bytes):
    s.sendto(command, RADAR_ADDR)
    logging.debug(f'Sent {len(command)} bytes to {RADAR_ADDR}')

def radar_stay_alive(s: socket):
    transmit_command(s, stay_alive_1sec)
    if (radar_stay_alive.counter == 0): transmit_command(s, rd_msg_5s)
    radar_stay_alive.counter = (radar_stay_alive.counter + 1)%5
    logging.debug(f'Sent stay alive command')
radar_stay_alive.counter = 0

def radar_tx_on(s: socket):
    transmit_command(s, rd_msg_tx_on)
    logging.debug(f'Sent tx on command')

def radar_tx_off(s: socket):
    transmit_command(s, rd_msg_tx_off)
    logging.debug(f'Sent tx off command')
