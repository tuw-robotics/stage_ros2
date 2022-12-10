#include <memory>
#include "stage_ros2/stage_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StageNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
