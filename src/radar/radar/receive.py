# ping pong client
import socket
import select
import radar.control as control
import logging

def main():
    logging.info(f'Run radar receive')
    with socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP) as m_comm_socket:
        m_comm_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        m_comm_socket.setblocking(0) # non-blocking socket
        
        while True:
            ready = select.select([m_comm_socket], [], [], control.TIMEOUT_IN_SECONDS)
            logging.debug(f'{ready=}')
            if ready[0]:
                data, addr = m_comm_socket.recvfrom(2048)
                logging.info(f'received {len(data)} bytes')
                logging.info(f'{data}')
            control.radar_stay_alive(m_comm_socket)


if __name__ == '__main__':
    main()
