#include <memory>
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/videoio.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <arpa/inet.h>

#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstring>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"

class VideoPublisher : public rclcpp::Node  
{
public:
    VideoPublisher() : Node("video_publisher"), sockfd(-1)
    {
        // this->declare_parameter<std::string>("client_ip", "10.223.75.168");
        this->declare_parameter<std::string>("client_ip", "127.0.0.1");
        this->declare_parameter<int>("client_port", 9000);

        this->get_parameter("client_ip", client_ip);
        this->get_parameter("client_port", client_port);

        init_udp_socket();
    }
    ~VideoPublisher()
    {
        if(sockfd != -1)
        {
            close(sockfd);
        }
    }

private:
    void init_udp_socket() 
    {
        if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed.");
            exit(EXIT_FAILURE);
        }
        
        memset(&cliaddr, 0, sizeof(cliaddr));
        cliaddr.sin_family = AF_INET;
        cliaddr.sin_port = htons(client_port);
        inet_pton(AF_INET, client_ip.c_str(), &cliaddr.sin_addr);

        send_video();
    }

    void hello_sender()
    {
        int i = 0;
        while (rclcpp::ok())
        {
            std::string message = "Hello World " + std::to_string(i++);
            RCLCPP_INFO(this->get_logger(), "Sending: %s", message.c_str());
            sendto(sockfd, message.c_str(), message.length(), 0, (const struct sockaddr *)&cliaddr, sizeof(cliaddr));
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    void send_video()
    { 
        // cv::VideoCapture video_capture_(ament_index_cpp::get_package_share_directory("video_stream") + "/resource/testvideo.mp4");
        cv::VideoCapture video_capture_(0);
        cv::Mat frame;
        std::vector<uchar> buffer;
        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 80};
        double fps = video_capture_.get(cv::CAP_PROP_FPS);
        auto start = std::chrono::steady_clock::now();
        //log it
        RCLCPP_INFO(this->get_logger(), "Input Video: %fFPS", fps);
        while (rclcpp::ok() && video_capture_.isOpened())
        {
            if(video_capture_.read(frame)) {
                auto now = std::chrono::steady_clock::now();
                std::chrono::duration<double> elapsed = now - start;
                int minutes = static_cast<int>(elapsed.count()) / 60;
                int seconds = static_cast<int>(elapsed.count()) % 60;

                RCLCPP_INFO(this->get_logger(), "Streaming... Time Elapsed: %02d:%02d", minutes, seconds);

                cv::imencode(".jpg", frame, buffer, compression_params);
                sendto(sockfd, buffer.data(), buffer.size(), 0, (const struct sockaddr *)&cliaddr, sizeof(cliaddr));
                std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(1000000 / fps)));
            } else {
                std::string end_msg = "END OF STREAM";
                sendto(sockfd, end_msg.c_str(), end_msg.length(), 0, (const struct sockaddr *)&cliaddr, sizeof(cliaddr));
                RCLCPP_INFO(this->get_logger(), end_msg.c_str());  
                break;
                // rclcpp::shutdown();
                // return;     
            }      
        }
    }
    int sockfd;
    struct sockaddr_in cliaddr;
    std::string client_ip;
    int client_port;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisher>());
    rclcpp::shutdown();
    return 0;
}
