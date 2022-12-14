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

  void init(int argc, char **argv, bool gui, const char *fname, bool use_model_names);
  void start();
private:
  void on_timer();
  size_t count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  double value_double;
  int value_int;
  std::string world_file_;
  // The main simulator object
  Stg::World *stage_;

  // Last time that we received a command
  rclcpp::Time base_last_cmd_;
  rclcpp::Duration base_watchdog_timeout_{0, 0};

  void init_parameter();
  void callback_update_parameter();
  void callback_world();

  /***
   * @return false to indicate that we want to be called again
  */
  static bool s_update(Stg::World *world, StageNode *node)
  {
    node->callback_world();
    return false;
  }
    
  static void ghfunc(Stg::Model* mod, StageNode* node);

  std::shared_ptr<std::thread> thread_stage_; 
};

#endif // STAGE_ROS2_PKG__STAGE_ROS_HPP_
