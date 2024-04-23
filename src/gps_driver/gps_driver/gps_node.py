import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from gps import *

class GPSDriverNode(Node):
    def __init__(self):
        super().__init__('gps_driver')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_data', 10)
        self.timer = self.create_timer(1.0, self.publish_gps_data)
        #gps daemon running on local host with port 2947
        self.gps_session = gps(mode=WATCH_ENABLE)
        self.get_logger().info(f"gps_session created with mode={WATCH_ENABLE}")

    def publish_gps_data(self):
        try:
            report = next(self.gps_session)
            # print(f"GPS data: {report}\n\n")
            if report['class'] == 'TPV':
                if hasattr(report, 'lat') and hasattr(report, 'lon'):
                    gps_msg = NavSatFix()
                    gps_msg.latitude = report.lat
                    gps_msg.longitude = report.lon
                    if hasattr(report, 'alt'):
                        gps_msg.altitude = report.alt
                    self.publisher_.publish(gps_msg)
                    self.get_logger().info(f"Published real GPS data:\n{gps_msg}")
                else:
                    gps_msg = NavSatFix()
                    # hard coded for testing indoor
                    gps_msg.latitude = 111.11
                    gps_msg.longitude = 222.22
                    gps_msg.altitude = 333.33
                    self.publisher_.publish(gps_msg)
                    self.get_logger().info(f"Published fake GPS data:\n{gps_msg}")
        except StopIteration:
            self.get_logger().warn('GPSD has terminated')

def main(args=None):
    rclpy.init(args=args)
    node = GPSDriverNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
