#include <memory>
#include "stage_ros2/stage_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StageNode>(rclcpp::NodeOptions());
  node->init(argc - 1, argv);
  if (node->SubscribeModels() != 0) {exit(-1);}
  std::thread t = std::thread([&node]() {rclcpp::spin(node);});
  node->world->Start();
  Stg::World::Run();
  return 0;
}
