# ping pong client
import socket
import radar.control as control
import logging

def main():
    with socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP) as m_comm_socket:
        m_comm_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        control.radar_tx_on(m_comm_socket)
        logging.info(f'Turn on radar')

if __name__ == '__main__':
    main()
