#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <cmath>

#define size_thresh 50
#define tolerance 10

class ObjectDetectionNode : public rclcpp::Node
{
public:
    ObjectDetectionNode()
        : Node("shape_filter_node")
    {
        // Subscribe to filtered RGB image topic
        filtered_rgb_subscription_ = create_subscription<sensor_msgs::msg::Image>(
            "/filtered_rgb", 10, std::bind(&ObjectDetectionNode::filtered_rgb_callback, this, std::placeholders::_1));

        // Subscribe to depth image topic
        depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
            "/depth/image_raw", 10, std::bind(&ObjectDetectionNode::depth_callback, this, std::placeholders::_1));

        // Publish filtered shape to filtered circles topic
        filtered_circles_publisher_ = create_publisher<sensor_msgs::msg::Image>(
            "/filtered_shapes", 10);
    }

private:
    std::vector<cv::Vec3f> centers;
    void filtered_rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Convert ROS image message to OpenCV format
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(cv_ptr->image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Iterate through the contours
    for (size_t i = 0; i < contours.size(); i++)
    {
        // Calculate bounding box for each contour
        cv::Rect bounding_box = cv::boundingRect(contours[i]);
        uint16_t prev_depth = 0;

        // Check if the bounding box is of the given size
        if(bounding_box.width > size_thresh && bounding_box.height > size_thresh){
            // Check if the bounding box is roughtly a box
            if(std::abs(bounding_box.width - bounding_box.height) < tolerance ){
                // Draw bounding box
                cv::rectangle(cv_ptr->image, bounding_box, cv::Scalar(255), 2); // Draw in white

                // Calculate center of the bounding box
                cv::Point center(bounding_box.x + bounding_box.width/2, bounding_box.y + bounding_box.height/2);

                // Extract depth at circle center
                uint16_t depth = cv_ptr->image.at<uint16_t>(center);
                depth = (depth==0) ? prev_depth : depth;
                prev_depth = depth;

                // Convert depth to string
                std::string depth_str = "Depth: " + std::to_string(depth) + " mm";

                // Draw text on top of bounding box
                cv::putText(cv_ptr->image, depth_str, cv::Point(bounding_box.x, bounding_box.y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);

                RCLCPP_INFO(get_logger(), depth_str);
            }

        }
    }


    // Publish the modified image
    filtered_circles_publisher_->publish(*cv_ptr->toImageMsg());
}


    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr filtered_rgb_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_circles_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
