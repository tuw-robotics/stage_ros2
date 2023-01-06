#include <stage_ros2/stage_node.hpp>

#include <chrono>
#include <memory>
#include <filesystem>

#define IMAGE "image"
#define DEPTH "depth"
#define CAMERA_INFO "camera_info"

using std::placeholders::_1;

StageNode::Vehicle::Camera::Camera(Stg::ModelCamera *m, std::shared_ptr<Vehicle> &v, StageNode *n)
    : model(m), vehicle(v), node(n){};
void StageNode::Vehicle::Camera::init(bool add_id_to_topic)
{
    model->Subscribe();
    if (add_id_to_topic)
    {
        topic_name_image = vehicle->name_space_ + IMAGE + std::to_string(id);
        topic_name_camera_info = vehicle->name_space_ + CAMERA_INFO + std::to_string(id);
        topic_name_depth = vehicle->name_space_ + DEPTH + std::to_string(id);
    }
    else
    {
        topic_name_image = vehicle->name_space_ + IMAGE;
        topic_name_camera_info = vehicle->name_space_ + CAMERA_INFO;
        topic_name_depth = vehicle->name_space_ + DEPTH;
    }
    pub_image = node->create_publisher<sensor_msgs::msg::Image>(topic_name_image, 10);
    pub_camera = node->create_publisher<sensor_msgs::msg::CameraInfo>(topic_name_camera_info, 10);
    pub_depth = node->create_publisher<sensor_msgs::msg::Image>(topic_name_depth, 10);
}
void StageNode::Vehicle::Camera::publish_msg()
{
}
void StageNode::Vehicle::Camera::publish_tf()
{
}