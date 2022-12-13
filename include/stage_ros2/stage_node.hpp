#ifndef STAGE_ROS2_PKG__STAGE_ROS_HPP_
#define STAGE_ROS2_PKG__STAGE_ROS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "stage_ros2/visibility.h"

#include <stage.hh>

class StageNode : public rclcpp::Node
{
public:
  STAGE_ROS2_PACKAGE_PUBLIC StageNode(rclcpp::NodeOptions options);

  void init(int argc, char** argv, bool gui, const char* fname, bool use_model_names);
private:
  void on_timer();
  size_t count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  double value_double;
  int value_int;
  // The main simulator object
  Stg::World* world;
    rclcpp::Service<StageNode::srv::ControlParameter>::SharedPtr srv_control_param_request_;
  void init_parameter();
  void callback_update_parameter();
};

#endif  // STAGE_ROS2_PKG__STAGE_ROS_HPP_
