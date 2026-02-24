#ifndef STAGE_ROS2_PKG__STAGE_ROS_HPP_
#define STAGE_ROS2_PKG__STAGE_ROS_HPP_
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <stage_ros2/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>

// libstage
#include <stage.hh>

#include "stage_ros2/visibility.h"
#include "stage_ros2/vehicle.hpp"



namespace stage_ros2 {

// Our node
class StageNode : public rclcpp::Node
{
public:
  STAGE_ROS2_PACKAGE_PUBLIC StageNode(rclcpp::NodeOptions options);

public:
  // A mutex to lock access to fields that are used in message callbacks
  std::mutex msg_lock;

  bool isDepthCanonical_;                  /// ROS parameter
  bool use_stamped_velocity_;              /// ROS parameter
  bool use_ackermann_;                     /// ROS parameter
  bool use_static_transformations_;        /// ROS parameter
  std::string frame_id_odom_name_;         /// ROS parameter
  std::string frame_id_world_name_;        /// ROS parameter
  std::string frame_id_base_link_name_;    /// ROS parameter

private:
  /// vector to hold the simulated vehicles with ros interfaces
  std::vector<std::shared_ptr<Vehicle>> vehicles_;

  bool enforce_prefixes_;                  /// ROS parameter
  bool one_tf_tree_;                       /// ROS parameter
  bool enable_gui_;                        /// ROS parameter
  bool publish_ground_truth_;              /// ROS parameter
  std::string world_file_;                 /// ROS parameter

  // TF broadcaster to publish the robot odom
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_stage_;

  // Service to listening on soft reset signals
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset_;

  // publisher for the simulated clock
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

  /// called only ones to init the models and to crate for each model a link to ROS
  static int callback_init_stage_model(Stg::Model * mod, StageNode * node);

  /// called on every simulation interation
  static int callback_update_stage_world(Stg::World * world, StageNode * node);

public:
  ~StageNode();
  // Constructor
  void init(int argc, char ** argv);

  // declares ros parameters
  void declare_parameters();

  // int ros parameters for the startup
  void update_parameters();

  // callback to check changes on the parameters
  void callback_update_parameters();

  // timer to check regulary for parameter changes
  rclcpp::TimerBase::SharedPtr timer_update_parameter_;

  // Subscribe to models of interest.  Currently, we find and subscribe
  // to the first 'laser' model and the first 'position' model.  Returns
  // 0 on success (both models subscribed), -1 otherwise.
  int SubscribeModels();

  // Do one update of the world.  May pause if the next update time
  // has not yet arrived.
  bool UpdateWorld();

  // Service callback for soft reset
  bool cb_reset_srv(const std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr);

  // The main simulator object
  Stg::World * world;

  rclcpp::Duration base_watchdog_timeout_;

  // Current simulation time
  rclcpp::Time sim_time_;

public:
  static geometry_msgs::msg::TransformStamped create_transform_stamped(
    const tf2::Transform & in,
    const rclcpp::Time & timestamp, const std::string & frame_id,
    const std::string & child_frame_id);
    static geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);
    static geometry_msgs::msg::Pose createGeometryPose(const Stg::Pose &src);

};

}  // namespace stage_ros2

#endif // STAGE_ROS2_PKG__STAGE_ROS_HPP_
