#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
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

        // Publish filtered RGB image topic
        filtered_rgb_publisher_ = create_publisher<sensor_msgs::msg::Image>(
            "/filtered_rgb", 10);
    }

private:
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
        cv::Mat hsv;
        cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

        // Define lower and upper bounds for white color in HSV
        cv::Scalar lower_white(0, 0, 200);
        cv::Scalar upper_white(180, 255, 255);

        // Create a mask to filter out white pixels
        cv::Mat mask;
        cv::inRange(hsv, lower_white, upper_white, mask);

        // Apply the mask to the original image
        cv::Mat filtered_image;
        cv_ptr->image.copyTo(filtered_image, mask);

        // Publish filtered RGB image
        auto filtered_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, filtered_image).toImageMsg();
        filtered_rgb_publisher_->publish(*filtered_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_rgb_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KinectFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
