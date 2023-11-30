#include <map>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class KinectFilterNode : public rclcpp::Node
{
public:
    KinectFilterNode()
        : Node("color_filter_node")
    {
        // Subscribe to RGB image topic
        rgb_subscription_ = create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&KinectFilterNode::rgb_callback, this, std::placeholders::_1));

        // Subscribe to RGB image topic
        rgb_mask_mode_subscription_ = create_subscription<std_msgs::msg::String>(
            "/color_control", 10, std::bind(&KinectFilterNode::color_mode_callback, this, std::placeholders::_1));

        // Publish filtered RGB image topic
        filtered_rgb_publisher_ = create_publisher<sensor_msgs::msg::Image>(
            "/filtered_rgb", 10);

        // Initialize filter thresholds in the constructor
        filter_thresholds["r1"] = std::make_pair(cv::Scalar(0, 100, 20), cv::Scalar(10, 255, 255));
        filter_thresholds["r2"] = std::make_pair(cv::Scalar(160, 100, 20), cv::Scalar(180, 255, 255));
        filter_thresholds["bl"] = std::make_pair(cv::Scalar(100, 50, 50), cv::Scalar(140, 255, 255));

        // Initialize the final mask with an identity matrix
        final_mask = cv::Mat::zeros(480, 640, CV_8U);
    }

private:
    // Mask variables
    // Dictionary to map all the colors
    std::map<std::string, std::pair<cv::Scalar, cv::Scalar>> filter_thresholds;

    // HSV representation of image
    cv::Mat hsv;

    // Initial mask
    cv::Mat final_mask;

    // Buffer to hold current filters
    std::vector<std::string> filter_tokens;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rgb_mask_mode_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_rgb_publisher_;

    void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS image message to OpenCV format
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Convert RGB image to HSV color space
        cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

        // Apply the mask to the original image
        this->final_mask = cv::Mat::zeros(480, 640, CV_8U);

        for (auto ele : filter_tokens)
        {
            cv::Mat mask;
            cv::inRange(hsv, filter_thresholds[ele].first, filter_thresholds[ele].second, mask);
            this->final_mask += mask;
        }

        cv::Mat filtered_image;
        cv_ptr->image.copyTo(filtered_image, final_mask);

        // Publish filtered RGB image
        auto filtered_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, filtered_image).toImageMsg();
        filtered_rgb_publisher_->publish(*filtered_msg);
    }

    void color_mode_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        size_t pos = 0;
        size_t initialPos = 0;
        char ch=',';
        filter_tokens.clear();
        std::string txt = msg->data.c_str(); 

        // Decompose statement
        RCLCPP_INFO(this->get_logger(), "Received string: [%d]", txt.find(ch, initialPos)!= std::string::npos);
        while ((pos = txt.find(ch, initialPos)) != std::string::npos)
        {
            filter_tokens.push_back(txt.substr(initialPos, pos - initialPos));
            initialPos = pos + 1;
            RCLCPP_INFO(this->get_logger(), "POS: [%d]", pos);
        }

        for (auto ele : filter_tokens)
        {
            cv::Mat mask;
            RCLCPP_INFO(this->get_logger(), "Mask contorl: [%s]", ele);
        }

        RCLCPP_INFO(this->get_logger(), "Received string: [%s]", msg->data.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KinectFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
