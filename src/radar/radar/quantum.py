import socket
import struct

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from radar_interfaces.srv import RadarData  

from radar.packets.location_info import LocationInfo
from radar.packets import control_message


class Qauntum(Node):

    def __init__(self):
        super().__init__('quantum')
        # uncomment below to simulate locator
        # data = bytes.fromhex('00000000928b80cb28000000030064000608100001b301e80e0a11007501a8c00f0a3600')
        # bl = LocationInfo.parse(data[:36])
        # self.quantum_location = LocationInfo(*bl)
        self.quantum_location = self.locate_quantum() # this might take a while to return
        
        self.command_socket = self.create_command_socket() # socket for controlling radar
        self.report_socket = self.create_report_socket() # socket for receiving radar data
        
        # keey radar alive
        self.get_logger().info(f'Keeping radar alive')
        self.standby_timer = self.create_timer(1, self.standby_timer_callback) # 1 second timer
        self.alive_counter = 0
        
        # subscription
        self.radar_control_subscription = self.create_subscription(
            String,
            'radar_control',
            self.radar_control_callback,
            10)
        
        # service
        self.radar_data_srv = self.create_service(
            RadarData, 
            'radar_data', 
            self.radar_data_callback)


    def locate_quantum(self, multicast_group='224.0.0.1', multicast_port=5800, quantum_model_id=40) -> LocationInfo:
        # Create UDP socket
        locator_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP)
        locator_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Bind to the server address
        # on this port, receives ALL multicast groups
        locator_socket.bind(('', multicast_port))
        
        # Tell the operating system to add the socket to the multicast group
        # on all interfaces.
        mreq = struct.pack('4sL', socket.inet_aton(multicast_group), socket.INADDR_ANY)
        locator_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        self.get_logger().info(f'Initialized locator socket')
        
        # loop until Raymarine Quantum is found
        while True:
            self.get_logger().info(f'Locating radar...')
            data, senderaddr = locator_socket.recvfrom(1024)
            self.get_logger().debug(f'received ({len(data)} bytes) from {senderaddr}')
            if len(data) != 36: continue # ignore any packets not 36 bytes
            
            quantum_location = LocationInfo(*LocationInfo.parse(data))
            self.get_logger().debug(f'{quantum_location=}')
            if quantum_location.model_id != quantum_model_id: continue
            
            self.get_logger().info(f'Found radar at {quantum_location.radar_ip}')
            break
        
        self.get_logger().info(f'Closing locator socket')
        locator_socket.close()
        return quantum_location


    def create_command_socket(self) -> socket:
        command_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP)
        command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        command_socket.setblocking(0) # non-blocking socket
        self.get_logger().info(f'Initialized command socket')
        
        return command_socket


    def create_report_socket(self) -> socket:
        report_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM, proto=socket.IPPROTO_UDP)
        report_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Bind to the server address
        # on this port, receives ALL multicast groups
        report_socket.bind(('', self.quantum_location.data_port))
        # Tell the operating system to add the socket to the multicast group
        # on all interfaces.
        # mreq = struct.pack('4sL', socket.inet_aton(self.quantum_location.data_ip), socket.INADDR_ANY)
        mreq = struct.pack('4s4s', socket.inet_aton(self.quantum_location.data_ip), socket.inet_aton("192.168.2.2")) # set to the ip of the receiver
        report_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        self.get_logger().info(f'Initialized report socket')
        
        return report_socket


    def standby_timer_callback(self):
        self.radar_stay_alive()
        self.alive_counter = (self.alive_counter + 1)%5


    def radar_stay_alive(self) -> None:
        self.transmit_command(control_message.STAY_ALIVE_1SEC)
        if (self.alive_counter%5 == 0):
            self.transmit_command(control_message.STAY_ALIVE_5SEC)
        self.get_logger().info(f'Sent stay alive command')


    def transmit_command(self, command) -> None:
        self.command_socket.sendto(command, (self.quantum_location.radar_ip, self.quantum_location.radar_port))
        self.get_logger().debug(f'Sent ({len(command)} bytes) to {self.quantum_location.radar_ip,}:{self.quantum_location.radar_port}')


    def radar_tx_on(self) -> None:
        self.transmit_command(control_message.TX_ON)
        self.get_logger().info(f'Sent tx on command')


    def radar_tx_off(self) -> None:
        self.transmit_command(control_message.TX_OFF)
        self.get_logger().info(f'Sent tx off command')


    def radar_control_callback(self, msg):
        match msg.data:
            case 'start_scan':
                self.radar_tx_on()
            case 'stop_scan':
                self.radar_tx_off()
            case _:
                self.radar_tx_off()


    def radar_data_callback(self, request, response):
        try:
            data, senderaddr = self.report_socket.recvfrom(1024)
        except OSError:
            return None
        
        response.data = data
        return response


def main(args=None):
    rclpy.init(args=args)

    quantum = Qauntum()

    rclpy.spin(quantum)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    quantum.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
