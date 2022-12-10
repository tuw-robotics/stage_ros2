#include <chrono>

#include "stage_ros2/stage_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

PublisherNode::PublisherNode(rclcpp::NodeOptions options)
: Node("publisher_node", options), count_(0)
{
  publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = create_wall_timer(
    500ms, std::bind(&PublisherNode::on_timer, this));
}

void PublisherNode::on_timer()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publisher: '%s'", message.data.c_str());
  publisher_->publish(message);
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(PublisherNode)