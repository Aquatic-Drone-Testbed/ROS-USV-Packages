# ping pong client
import socket
import select
import radar.control as control
import logging
from threading import Thread
import time

def receive(s: socket):
    while True:
        ready = select.select([s], [], [], control.TIMEOUT_IN_SECONDS)
        logging.debug(f'{ready=}')
        if ready[0]:
            data, addr = s.recvfrom(2048)
            logging.info(f'received {len(data)} bytes from {addr}')
            logging.info(f'{data}')
        control.radar_stay_alive(s)

def main():
    logging.info(f'Run radar receive')
    with socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP) as m_comm_socket:
        m_comm_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        m_comm_socket.setblocking(0) # non-blocking socket
        logging.info(f'initialized {m_comm_socket=}')
        
        control.radar_stay_alive(m_comm_socket)
        recv_thread = Thread(target = receive, args=(m_comm_socket, ))
        recv_thread.start()
        time.sleep(0.5)
        control.radar_tx_on(m_comm_socket)
        time.sleep(3600)
        control.radar_tx_off(m_comm_socket)
        time.sleep(5)


if __name__ == '__main__':
    main()
