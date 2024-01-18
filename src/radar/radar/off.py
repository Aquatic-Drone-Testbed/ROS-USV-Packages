# send radar off command
import socket
import radar.control as control
import logging

def main():
    with socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP) as m_comm_socket:
        m_comm_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        logging.info(f'initialized {m_comm_socket=}')
        control.radar_tx_off(m_comm_socket)
        logging.info(f'Turn off radar')

if __name__ == '__main__':
    main()
