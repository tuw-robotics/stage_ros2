#include "stage_ros2/fiducial_detector.hpp"
#include "stage_ros2/stage_node.hpp"

#include <memory>

namespace stage_ros2 {

#define TOPIC_FIDUCIALS "fiducials"
#define FRAME_FIDUCIALS "fiducial_sensor"

using std::placeholders::_1;

FiducialDetector::FiducialDetector(
  unsigned int id, Stg::ModelFiducial * m,
  std::shared_ptr<Vehicle> & v)
: initialized_(false), id_(id), model(m), vehicle(v) {}

unsigned int FiducialDetector::id() const
{
  return id_;
}
void FiducialDetector::init(bool add_id_to_topic)
{
  if(initialized_) return; 
  model->Subscribe();
  topic_name = vehicle->topic_name_space_ + TOPIC_FIDUCIALS;
  frame_id = vehicle->frame_name_space_ + FRAME_FIDUCIALS;
  if (add_id_to_topic) {
    topic_name += std::to_string(id());
    frame_id += std::to_string(id());
  }

  pub = vehicle->node()->create_publisher<marker_msgs::msg::MarkerDetection>(topic_name, 10);
  initialized_ = true;
}

bool FiducialDetector::prepare_msg()
{
  if (msg) {return true;}
  
  msg = std::make_shared<marker_msgs::msg::MarkerDetection>();
  msg->fov_horizontal = model->fov;
  msg->fov_vertical = model->fov;
  msg->distance_min = model->min_range;
  msg->distance_max = model->max_range_anon;
  msg->distance_max_id = model->max_range_id;
  msg->view_direction.x = 0.0;
  msg->view_direction.y = 0.0;
  msg->view_direction.z = 0.0;
  msg->view_direction.w = 1.0;
  msg->type = "stage";
  msg->markers.clear();
  msg->header.frame_id = frame_id;

  return true;
}

bool FiducialDetector::prepare_tf()
{

  transform = std::make_shared<geometry_msgs::msg::TransformStamped>();

  Stg::Pose pose = model->GetPose();
  tf2::Quaternion quternion;
  quternion.setRPY(0.0, 0.0, pose.a);
  tf2::Transform txLaser =
    tf2::Transform(
    quternion,
    tf2::Vector3(pose.x, pose.y, vehicle->positionmodel->GetGeom().size.z + pose.z));
  *transform = StageNode::create_transform_stamped(
    txLaser, vehicle->node()->sim_time_, vehicle->frame_id_base_link_,
    frame_id);
  if (vehicle->node()->use_static_transformations_) {
    vehicle->tf_static_broadcaster_->sendTransform(*transform);
  }
  return true;
}

void FiducialDetector::publish_msg()
{
  // Guard 
  if(!initialized_) return; 

  if (model->GetFiducials().size() == 0) {
    RCLCPP_WARN(vehicle->node()->get_logger(), "We only send a detection if at least one fiducial was detected");
    return;
  }
  

  if (prepare_msg()) {
    msg->header.stamp = vehicle->node()->sim_time_; 
    msg->markers.clear();
    auto &fiducials = model->GetFiducials();  
    for (const Stg::ModelFiducial::Fiducial &fiducial: fiducials) {
      marker_msgs::msg::Marker marker;
      if(fiducial.id != -1){
        marker.ids.push_back(fiducial.id);
        marker.ids_confidence.push_back(1.0);
        marker.pose.position.x = fiducial.range * cos(fiducial.bearing);
        marker.pose.position.y = fiducial.range * sin(fiducial.bearing);
        marker.pose.orientation = StageNode::createQuaternionMsgFromYaw(fiducial.geom.a);
      }
      msg->markers.push_back(std::move(marker));
    }
    
    pub->publish(*msg);
  }
}

void FiducialDetector::publish_tf()
{
  if (prepare_tf()) {

    if (vehicle->node()->use_static_transformations_) {return;}

    // use tf publsiher only if use_static_transformations_ is false
    transform->header.stamp = vehicle->node()->sim_time_;
    vehicle->tf_broadcaster_->sendTransform(*transform);
  }
}

}  // namespace stage_ros2
