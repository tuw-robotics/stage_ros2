#ifndef STAGE_ROS2_PKG__FIDUCIAL_DETECTOR_HPP_
#define STAGE_ROS2_PKG__FIDUCIAL_DETECTOR_HPP_

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <marker_msgs/msg/marker_detection.hpp>

// libstage
#include <stage.hh>

namespace stage_ros2 {

// Forward declaration to avoid circular dependency
class Vehicle;

class FiducialDetector
{
  bool initialized_;
  size_t id_;
  Stg::ModelFiducial * model;
  std::shared_ptr<Vehicle> vehicle;
  std::string topic_name;
  std::string frame_base;
  std::string frame_id;
  geometry_msgs::msg::TransformStamped::SharedPtr transform;
  rclcpp::Publisher<marker_msgs::msg::MarkerDetection>::SharedPtr pub;
  marker_msgs::msg::MarkerDetection::SharedPtr msg;
  bool prepare_msg();
  bool prepare_tf();

public:
  FiducialDetector(
    unsigned int id, Stg::ModelFiducial * m, std::shared_ptr<Vehicle> & vehicle);
  void init(bool add_id_to_topic);
  unsigned int id() const;
  void publish_msg();
  void publish_tf();
};

}  // namespace stage_ros2

#endif  // STAGE_ROS2_PKG__FIDUCIAL_DETECTOR_HPP_
