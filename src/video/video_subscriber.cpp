#include <memory>
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <arpa/inet.h>

#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstring>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"

class VideoSubscriber : public rclcpp::Node
{
public:
    VideoSubscriber() : Node("video_subscriber"), sockfd(-1)
    {
        this->declare_parameter<int>("server_port", 9000);
        this->get_parameter("server_port", server_port);

        init_udp_socket();
    }
    ~VideoSubscriber() 
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
        
        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(server_port);
        servaddr.sin_addr.s_addr = INADDR_ANY;

        if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Bind failed");
            exit(EXIT_FAILURE);
        }
        receive_video();
    }

    void hello_receiver() 
    {
        char buffer[1024];
        struct sockaddr_in cliaddr;
        unsigned int len = sizeof(cliaddr);

        while (rclcpp::ok()) 
        {
            int n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&cliaddr, &len);
            buffer[n] = '\0';

            std::string received_msg(buffer);
            RCLCPP_INFO(this->get_logger(), "Received message: %s", received_msg.c_str());
        }
    }

    void receive_video() 
    {   
        char buffer[65507];
        struct sockaddr_in cliaddr;
        unsigned int len = sizeof(cliaddr);

        while (rclcpp::ok()) 
        {
            int n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&cliaddr, &len);
            buffer[n] = '\0';

            std::string received_msg(buffer);
            RCLCPP_INFO(this->get_logger(), "Received message: %s", received_msg.c_str());

            if(received_msg == "END OF STREAM") {
                RCLCPP_INFO(this->get_logger(), "End of stream detected.");
                break;
            } else {
                std::vector<uchar> data(buffer, buffer+n);
                cv::Mat frame = cv::imdecode(cv::Mat(data), cv::IMREAD_COLOR);
                
                if(!frame.empty()) 
                {
                    cv::imshow("Received Frame", frame);
                    cv::waitKey(1);
                }
            }
        }
    }

    int sockfd;
    struct sockaddr_in servaddr;
    int server_port;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto video_subscriber = std::make_shared<VideoSubscriber>();
    rclcpp::shutdown();
    return 0;
}