import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import roslibpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class Ros1ToRos2Bridge(Node):

    def __init__(self):
        super().__init__('ros1_to_ros2_bridge')
        
        self.ros = roslibpy.Ros(host='localhost', port=9090)
        self.ros.run()

        # QoS profile with RELIABLE reliability
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/ros2_radar_odom', qos_profile)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/ros2_radar_registered', qos_profile)

        # Subscribers to ROS1 topics via roslibpy
        self.odom_listener = roslibpy.Topic(self.ros, '/radar_odom', 'nav_msgs/Odometry')
        self.odom_listener.subscribe(self.odom_callback)

        self.pointcloud_listener = roslibpy.Topic(self.ros, '/radar_registered', 'sensor_msgs/PointCloud2')
        self.pointcloud_listener.subscribe(self.pointcloud_callback)

    def odom_callback(self, message):
        msg = Odometry()
        msg.header.stamp.sec = message['header']['stamp']['secs']
        msg.header.stamp.nanosec = message['header']['stamp']['nsecs']
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = message['pose']['pose']['position']['x']
        msg.pose.pose.position.y = message['pose']['pose']['position']['y']
        msg.pose.pose.position.z = message['pose']['pose']['position']['z']
        msg.pose.pose.orientation.x = message['pose']['pose']['orientation']['x']
        msg.pose.pose.orientation.y = message['pose']['pose']['orientation']['y']
        msg.pose.pose.orientation.z = message['pose']['pose']['orientation']['z']
        msg.pose.pose.orientation.w = message['pose']['pose']['orientation']['w']
        msg.pose.covariance = message['pose']['covariance']

        msg.twist.twist.linear.x = message['twist']['twist']['linear']['x']
        msg.twist.twist.linear.y = message['twist']['twist']['linear']['y']
        msg.twist.twist.linear.z = message['twist']['twist']['linear']['z']
        msg.twist.twist.angular.x = message['twist']['twist']['angular']['x']
        msg.twist.twist.angular.y = message['twist']['twist']['angular']['y']
        msg.twist.twist.angular.z = message['twist']['twist']['angular']['z']
        msg.twist.covariance = message['twist']['covariance']

        self.odom_pub.publish(msg)

    def pointcloud_callback(self, message):
        msg = PointCloud2()
        
        msg.header.stamp.sec = message['header']['stamp']['secs']
        msg.header.stamp.nanosec = message['header']['stamp']['nsecs']
        msg.header.frame_id = 'radar_frame'
        
        msg.height = message['height']
        msg.width = message['width']
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
    
        msg.is_bigendian = message['is_bigendian']
        msg.point_step = message['point_step']
        msg.row_step = message['row_step']
        
        data_bytes = message['data'].encode()
        msg.data = data_bytes
        msg.is_dense = message['is_dense']

        self.pointcloud_pub.publish(msg)

    def destroy_node(self):
        super().destroy_node()
        self.odom_listener.unsubscribe()
        self.pointcloud_listener.unsubscribe()
        self.ros.terminate()

def main(args=None):
    rclpy.init(args=args)
    node = Ros1ToRos2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
