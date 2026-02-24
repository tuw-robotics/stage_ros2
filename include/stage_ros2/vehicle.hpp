#ifndef STAGE_ROS2_PKG__VEHICLE_HPP_
#define STAGE_ROS2_PKG__VEHICLE_HPP_

#include <string>
#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <stage_ros2/transform_broadcaster.h>
#include <stage_ros2/static_transform_broadcaster.h>

// libstage
#include <stage.hh>

#include "stage_ros2/ranger.hpp"
#include "stage_ros2/fiducial_detector.hpp"
#include "stage_ros2/camera.hpp"

namespace stage_ros2 {

// Forward declaration to avoid circular dependency
class StageNode;

class Vehicle
{
public:

private:
  bool initialized_;
  size_t id_;
  Stg::Pose initial_pose_;
  std::string name_;     /// used for the ros publisher
  StageNode * node_;
  Stg::World * world_;
  rclcpp::Time time_last_cmd_received_;
  rclcpp::Time timeout_cmd_;        /// if no command is received befor the vehicle is stopped
  // Last time we saved global position (for velocity calculation).
  rclcpp::Time time_last_pose_update_;

  std::string topic_name_cmd_;
  std::string topic_name_drive_;

  std::string topic_name_tf_;
  std::string topic_name_tf_static_;
  std::string topic_name_odom_;
  std::string topic_name_ground_truth_;
  std::string frame_id_odom_;
  std::string frame_id_world_;
  nav_msgs::msg::Odometry msg_odom_;
  std::shared_ptr<Stg::Pose> global_pose_;

public:
  std::string topic_name_space_;
  std::string frame_name_space_;
  std::string frame_id_base_link_;
  Vehicle(size_t id, const Stg::Pose & pose, const std::string & name, StageNode * node);

  void soft_reset();
  size_t id() const;
  const std::string & name() const;
  const std::string & name_space() const;
  void init(bool use_topic_prefixes, bool use_one_tf_tree);
  void callback_cmd(const geometry_msgs::msg::Twist::SharedPtr msg);
  void callback_cmd_stamped(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void callback_drive(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg);
  void callback_drive_stamped(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
  void publish_msg();
  void publish_tf();
  void check_watchdog_timeout();
  StageNode * node()
  {
    return node_;
  }

  // stage related models
  Stg::ModelPosition * positionmodel;               // one position
  std::vector<std::shared_ptr<Ranger>> rangers_;     // multiple rangers per position
  std::vector<std::shared_ptr<FiducialDetector>> fiducial_detectors_;     // multiple fiducial detectors  per position
  std::vector<std::shared_ptr<Camera>> cameras_;      // multiple cameras per position

  // ros publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;                     // one odom
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_ground_truth_;             // one ground truth
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;                 // one cmd_vel subscriber
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_stamped_;  // one sub_cmd_stamped_ subscriber
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr sub_drive_;     // one drive subscriber
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr sub_drive_stamped_;  // one drive_stamped_ subscriber

  std::shared_ptr<stage_ros2::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::shared_ptr<stage_ros2::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace stage_ros2

#endif  // STAGE_ROS2_PKG__VEHICLE_HPP_
