#include <stage_ros2/stage_node.hpp>

#include <chrono>
#include <memory>
#include <filesystem>

StageNode::Vehicle::Vehicle(size_t id, Stg::Pose pose, rclcpp::Node *node)
    : id_(id), initial_pose_(pose), node_(node)
{
}

size_t StageNode::Vehicle::id() const{
    return id_;
}
void StageNode::Vehicle::soft_reset()
{
    positionmodel->SetPose(this->initial_pose_);
    positionmodel->SetStall(false);
}