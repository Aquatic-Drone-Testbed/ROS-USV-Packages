import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from gps import *

class GPSDriverNode(Node):
    def __init__(self):
        super().__init__('gps_driver')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.timer = self.create_timer(1.0, self.publish_gps_data)
        #gps daemon running on local host with port 2947
        self.gps_session = gps(mode=WATCH_ENABLE)
        self.get_logger().info(f"gps_session created with mode={WATCH_ENABLE}")

    def publish_gps_data(self):
        try:
            report = next(self.gps_session)
            # print(f"GPS data: {report}\n\n")
            if report['class'] == 'TPV' and hasattr(report, 'lat') and hasattr(report, 'lon') and report.mode >= 2:
                gps_msg = NavSatFix()
                gps_msg.header.stamp = self.get_clock().now().to_msg()
                gps_msg.header.frame_id = 'gps'  # Or whatever frame ID is appropriate
                gps_msg.latitude = report.lat
                gps_msg.longitude = report.lon
                if hasattr(report, 'alt') and report.mode == 3:
                    gps_msg.altitude = report.alt
                self.publisher_.publish(gps_msg)
                self.get_logger().info(f"Published real GPS data:\n{gps_msg}")
            else:
                self.get_logger().info(f"no fix")
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
            self.get_logger().warn('GPSD has terminated')

def main(args=None):
    rclpy.init(args=args)
    node = GPSDriverNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
