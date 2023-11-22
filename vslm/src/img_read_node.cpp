#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.

using std::placeholders::_1;

class MinimalImageReader : public rclcpp::Node
{
private:

    sensor_msgs::msg::Image::SharedPtr msg_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;  

public:

    MinimalImageReader() : Node("image_reader_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>("image", 10, std::bind(&MinimalImageReader::image_callback, this, _1));
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
}; 