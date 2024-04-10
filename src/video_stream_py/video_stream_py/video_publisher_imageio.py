import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import imageio.v3 as iio
import numpy as np

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')

        video_path = 0  # USB Camera Input
        self.video_reader = iio.imiter(f"<video{video_path}>")
        self.frame_generator = iter(self.video_reader)

        fps = 30
        # fps = self.video_capture.get(cv2.CAP_PROP_FPS) or 30 # Default to 30 FPS if not available
        self.get_logger().info(f'Input Video: {fps}FPS')

        self.publisher_ = self.create_publisher(Image, 'video_stream', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1 / fps, self.timer_callback)

    def timer_callback(self):
        try:
            # Read the next frame from the frame generator.
            frame = next(self.frame_generator)
        except StopIteration:
            # If there are no frames left, we assume the video is over.
            self.get_logger().info('End of video stream')
            self.destroy_node()
            rclpy.shutdown()
            return

        self.get_logger().info(f'Sending frame: Width = {frame.shape[1]}, Height = {frame.shape[0]}')
        image_message = self.bridge.cv2_to_imgmsg(np.asarray(frame), encoding='rgb8')
        self.publisher_.publish(image_message)

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()