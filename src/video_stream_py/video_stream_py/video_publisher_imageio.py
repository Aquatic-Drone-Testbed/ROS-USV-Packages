import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import imageio.v3 as iio
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher_node')
        
        video_path = 0  # USB Camera Input
        self.video_reader = iio.imiter(f"<video{video_path}>")
        self.frame_generator = iter(self.video_reader)

        fps = 30
        # fps = self.video_capture.get(cv2.CAP_PROP_FPS) or 30 # Default to 30 FPS if not available
        self.get_logger().info(f'Input Video: {fps}FPS')

        self.diagnostic_pub = self.create_publisher(String, 'diagnostic_status', 10)
        self.diagnostic_timer = self.create_timer(3.0, self.publish_video_heartbeat)
        self.camera_on = True # Camera is off by default

        self.publisher_ = self.create_publisher(Image, 'video_stream', 10)
        self.subscription = self.create_subscription(String, 'camera_control', self.listener_callback, 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1 / fps, self.timer_callback)

    def listener_callback(self, msg):
        # Toggle camera state
        if(msg.data == "CAM TOGGLE"):
            self.camera_on = not self.camera_on
            state = 'on' if self.camera_on else 'off'
            self.get_logger().info(f'Camera toggled {state}')

    def timer_callback(self):
        if not self.camera_on:
            self.get_logger().warn('Camera is off, not sending frame')
            return
            
        # remove the following lines when using a real camera
        elif self.camera_on:
            self.get_logger().debug('Camera is on, sending frame')

        try:
            # Read the next frame from the frame generator.
            frame = next(self.frame_generator)
        except StopIteration:
            # If there are no frames left, we assume the video is over.
            self.get_logger().warn('End of video stream')
            self.destroy_node()
            rclpy.shutdown()
            return

        self.get_logger().debug(f'Sending frame: Width = {frame.shape[1]}, Height = {frame.shape[0]}')
        image_message = self.bridge.cv2_to_imgmsg(np.asarray(frame), encoding='rgb8')
        self.publisher_.publish(image_message)

    def publish_video_heartbeat(self):
        msg = String()
        if self.camera_on:
            msg.data = "Camera: On"
        else:
            msg.data = "Camera: Off"
        
        self.diagnostic_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()