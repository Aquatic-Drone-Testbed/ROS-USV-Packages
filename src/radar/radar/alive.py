# Send radar keep alive command for 1 hour
import socket
import radar.control as control
import logging
from threading import Thread
import time

def keep_alive(s: socket):
    while True:
        time.sleep(1)
        control.radar_stay_alive(s)

def main():
    logging.info(f'Run radar receive')
    with socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP) as m_comm_socket:
        m_comm_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        m_comm_socket.setblocking(0) # non-blocking socket
        logging.info(f'initialized {m_comm_socket=}')
        
        control.radar_stay_alive(m_comm_socket)
        keep_alive_thread = Thread(target = keep_alive, args=(m_comm_socket, ))
        keep_alive_thread.start()
        keep_alive_thread.join(3600)


if __name__ == '__main__':
    main()
