#ifndef STAGE_ROS2_PKG__VEHICLE_HPP_
#define STAGE_ROS2_PKG__VEHICLE_HPP_

#include <string>
#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <stage_ros2/transform_broadcaster.h>
#include <stage_ros2/static_transform_broadcaster.h>

// libstage
#include <stage.hh>

// Forward declaration to avoid circular dependency
class StageNode;

class Vehicle
{
public:
  class Ranger
  {
    bool initialized_;
    size_t id_;
    Stg::ModelRanger * model;
    std::shared_ptr<Vehicle> vehicle;
    std::string topic_name;
    std::string frame_base;
    std::string frame_id;
    geometry_msgs::msg::TransformStamped::SharedPtr transform;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub;
    sensor_msgs::msg::LaserScan::SharedPtr msg;
    bool prepare_msg();
    bool prepare_tf();

public:
    Ranger(
      unsigned int id, Stg::ModelRanger * m, std::shared_ptr<Vehicle> & vehicle);
    void init(bool add_id_to_topic);
    unsigned int id() const;
    void publish_msg();
    void publish_tf();
  };

  class Camera
  {
    bool initialized_;
    size_t id_;
    Stg::ModelCamera * model;
    std::shared_ptr<Vehicle> vehicle;
    geometry_msgs::msg::TransformStamped::SharedPtr transform;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image;             // multiple images
    sensor_msgs::msg::Image::SharedPtr msg_image;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth;             // multiple depths
    sensor_msgs::msg::Image::SharedPtr msg_depth;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera;       // multiple cameras
    sensor_msgs::msg::CameraInfo::SharedPtr msg_camera;
    bool prepare_msg();
    bool prepare_msg_image();
    bool prepare_msg_depth();
    bool prepare_msg_camera();
    bool prepare_tf();

public:
    Camera(
      unsigned int id, Stg::ModelCamera * m, std::shared_ptr<Vehicle> & vehicle);
    void init(bool add_id_to_topic);
    unsigned int id() const;
    void publish_msg();
    void publish_tf();
    std::string topic_name_image;
    std::string topic_name_depth;
    std::string topic_name_camera_info;
    std::string frame_id;
  };

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

  std::string topic_name_space_;
  std::string frame_name_space_;
  std::string topic_name_cmd_;
  std::string topic_name_drive_;

  std::string topic_name_tf_;
  std::string topic_name_tf_static_;
  std::string topic_name_odom_;
  std::string topic_name_ground_truth_;
  std::string frame_id_odom_;
  std::string frame_id_world_;
  std::string frame_id_base_link_;
  nav_msgs::msg::Odometry msg_odom_;
  std::shared_ptr<Stg::Pose> global_pose_;

public:
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

#endif  // STAGE_ROS2_PKG__VEHICLE_HPP_
