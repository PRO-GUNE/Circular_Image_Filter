#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ColorFilterControlNode : public rclcpp::Node
{
public:
    ColorFilterControlNode()
        : Node("color_filter_control_node")
    {
        // Publish filtered RGB image topic
        color_filter_control_publisher_ = create_publisher<std_msgs::msg::String>(
            "/color_control", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&ColorFilterControlNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
      std::string user_input;
      std::cout << "Enter color filter: ";
      std::getline(std::cin, user_input);
      if(user_input=="")
        return;

      auto message = std_msgs::msg::String();
      message.data = user_input;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      color_filter_control_publisher_->publish(message);
    }
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr color_filter_control_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ColorFilterControlNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
