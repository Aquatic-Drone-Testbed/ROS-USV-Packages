# library for controlling the radar

import socket
import logging
import time

logger = logging.getLogger(__name__)

HOST = '192.168.1.117'        # The remote host
PORT = 2575                 # The same port as used by the server
RADAR_ADDR = (HOST, PORT)

RD_MSG_TX_ON = bytes([0x10, 0x00, 0x28, 0x00,
                      0x01,  # Control value at offset 4 : 0 - off, 1 - on
                      0x00, 0x00, 0x00])

RD_MSG_TX_OFF = bytes([0x10, 0x00, 0x28, 0x00,
                       0x00,  # Control value at offset 4 : 0 - off, 1 - on
                       0x00, 0x00, 0x00])

# Quantum 1 sec stay alive
STAY_ALIVE_1SEC = bytes([0x00, 0x00, 0x28, 0x00, 0x52, 0x61,
                         0x64, 0x61, 0x72, 0x00, 0x00, 0x00])

# Quantum stay alive 5 sec message, 36 bytes
RD_MSG_5S = bytes([0x03, 0x89, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                   0x9e, 0x03, 0x00, 0x00, 0xb4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

RD_MSG_SET_RANGE = bytes([0x01, 0x01, 0x28, 0x00, 0x00,
                          0x0f,  # Quantum range index at pos 5
                          0x00, 0x00])


def transmit_command(command_socket: socket, command_ip: str, command_port: int, command: bytes):
    command_socket.sendto(command, (command_ip, command_port))
    logger.debug(f'Sent ({len(command)} bytes) to {command_ip}:{command_port}')


def radar_tx_on(command_socket: socket, command_ip: str, command_port: int):
    transmit_command(command_socket, command_ip, command_port, RD_MSG_TX_ON)
    logger.info(f'Sent tx on command')


def radar_tx_off(command_socket: socket, command_ip: str, command_port: int):
    transmit_command(command_socket, command_ip, command_port, RD_MSG_TX_OFF)
    logger.info(f'Sent tx off command')


def radar_stay_alive(command_socket: socket, command_ip: str, command_port: int):
    transmit_command(command_socket, command_ip, command_port, STAY_ALIVE_1SEC)
    if (radar_stay_alive.counter == 0):
        transmit_command(command_socket, command_ip, command_port, RD_MSG_5S)
    radar_stay_alive.counter = (radar_stay_alive.counter + 1)%5
    logger.info(f'Sent stay alive command')
radar_stay_alive.counter = 0


def standby_radar(command_socket: socket, command_ip: str, command_port: int):
    while True:
        try:
            radar_stay_alive(command_socket, command_ip, command_port)
        except OSError:
            break
        time.sleep(1)
