#include <memory>
#include "stage_ros2/stage_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StageNode>(rclcpp::NodeOptions());
  node->init(argc-1,argv);
  std::thread t = std::thread([&node](){rclcpp::spin(node);});
  node->start();
  rclcpp::shutdown();
  return 0;
}
