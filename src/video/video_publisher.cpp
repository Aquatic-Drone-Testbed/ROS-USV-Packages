#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/videoio.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class VideoPublisher : public rclcpp::Node
{
public:
    VideoPublisher() 
    : Node("video_publisher"), 
    // video_capture_(ament_index_cpp::get_package_share_directory("image_stream") + "/resource/testvideo.mp4")
    video_capture_(0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("video_stream", 10);
        //get video fps
        double fps = video_capture_.get(cv::CAP_PROP_FPS);
        if(fps == 0)
            fps = 30; //default to 30fps (if not available in video metadata
        //log it
        RCLCPP_INFO(this->get_logger(), "Input Video: %fFPS", fps);
        //create a timer that will send video at the corresponding frame rate
        timer_ = this->create_wall_timer(
            std::chrono::nanoseconds(static_cast<int>(1000000000 / fps)),
            std::bind(&VideoPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        if (!video_capture_.read(frame)) {
            // Send a final message to indicate the end of the stream
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv::Mat()).toImageMsg();
            msg->header.frame_id = "end_of_stream"; // Unique identifier
            publisher_->publish(*msg);

            RCLCPP_INFO(this->get_logger(), "End of video stream");
            rclcpp::shutdown();
            return;
        }
        // Convert to a ROS2 sensor_msgs::msg::Image and publish
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture video_capture_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisher>());
    rclcpp::shutdown();
    return 0;
}