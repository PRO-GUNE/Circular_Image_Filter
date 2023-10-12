#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

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
            "/kinect_ros2/depth/image_raw", 10, std::bind(&ObjectDetectionNode::depth_callback, this, std::placeholders::_1));
    }

private:
    std::vector<cv::Vec3f> circles;

    void filtered_rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
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

        // Convert to grayscale for circle detection
        cv::Mat gray;
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

        // Apply Gaussian blur to reduce noise
        cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);

        cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows / 8, 200, 100, 0, 0);

        for (size_t i = 0; i < circles.size(); i++)
        {
            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);

            // Draw the circle center
            cv::circle(cv_ptr->image, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);

            // Draw the circle outline
            cv::circle(cv_ptr->image, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
        }

        // Display the image with circles
        cv::imshow("Circles Detected", cv_ptr->image);
        cv::waitKey(1);
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

        // Assuming 'circles' is a vector of detected circles
        for (size_t i = 0; i < circles.size(); i++)
        {
            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));

            // Extract depth at circle center
            uint16_t depth = cv_ptr->image.at<uint16_t>(center);

            RCLCPP_INFO(this->get_logger(), "Depth at circle center: %u mm", depth);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr filtered_rgb_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
