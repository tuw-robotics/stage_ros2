#ifndef STAGE_ROS2_PKG__CAMERA_HPP_
#define STAGE_ROS2_PKG__CAMERA_HPP_

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

// libstage
#include <stage.hh>

namespace stage_ros2 {

// Forward declaration to avoid circular dependency
class Vehicle;

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

}  // namespace stage_ros2

#endif  // STAGE_ROS2_PKG__CAMERA_HPP_
