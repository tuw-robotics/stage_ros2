#include <stage_ros2/stage_node.hpp>

#include <chrono>
#include <memory>
#include <filesystem>

#define IMAGE "image"
#define DEPTH "depth"
#define CAMERA_INFO "camera_info"
#define ODOM "odom"
#define BASE_SCAN "base_scan"
#define BASE_POSE_GROUND_TRUTH "base_pose_ground_truth"
#define CMD_VEL "cmd_vel"

using std::placeholders::_1;

StageNode::Vehicle::Vehicle(size_t id, const Stg::Pose &pose, const std::string &name, StageNode *node)
    : id_(id), initial_pose_(pose), name_(name), node_(node)
{
}

size_t StageNode::Vehicle::id() const
{
    return id_;
}
void StageNode::Vehicle::soft_reset()
{
    positionmodel->SetPose(this->initial_pose_);
    positionmodel->SetStall(false);
}

const std::string &StageNode::Vehicle::name() const
{
    return name_;
}

void StageNode::Vehicle::init_topics(bool use_model_name)
{
    std::string name_space_;
    if (use_model_name)
        name_space_ = name() + "/";

    odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>(name_space_ + ODOM, 10);
    ground_truth_pub = node_->create_publisher<nav_msgs::msg::Odometry>(name_space_ + BASE_POSE_GROUND_TRUTH, 10);
    cmdvel_sub = node_->create_subscription<geometry_msgs::msg::Twist>(name_space_ + CMD_VEL, 10, std::bind(&StageNode::Vehicle::callback_cmd, this, _1));

    for (size_t i = 0; i < rangers.size(); ++i)
    {
        if (rangers.size() == 1)
        {
            laser_pubs.push_back(node_->create_publisher<sensor_msgs::msg::LaserScan>(name_space_ + BASE_SCAN, 10));
        }
        else
        {
            laser_pubs.push_back(node_->create_publisher<sensor_msgs::msg::LaserScan>(name_space_ + BASE_SCAN + std::to_string(i), 10));
        }
    }

    for (size_t i = 0; i < cameras.size(); ++i)
    {
        if (cameras.size() == 1)
        {
            image_pubs.push_back(node_->create_publisher<sensor_msgs::msg::Image>(name_space_ + IMAGE, 10));
            depth_pubs.push_back(node_->create_publisher<sensor_msgs::msg::Image>(name_space_ + DEPTH, 10));
            camera_pubs.push_back(node_->create_publisher<sensor_msgs::msg::CameraInfo>(name_space_ + CAMERA_INFO, 10));
        }
        else
        {
            image_pubs.push_back(node_->create_publisher<sensor_msgs::msg::Image>(name_space_ + IMAGE + std::to_string(i), 10));
            depth_pubs.push_back(node_->create_publisher<sensor_msgs::msg::Image>(name_space_ + DEPTH + std::to_string(i), 10));
            camera_pubs.push_back(node_->create_publisher<sensor_msgs::msg::CameraInfo>(name_space_ + CAMERA_INFO + std::to_string(i), 10));
        }
    }
}

void StageNode::Vehicle::callback_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->positionmodel->SetSpeed(msg->linear.x,
                                  msg->linear.y,
                                  msg->angular.z);
    node_->base_last_cmd_ = node_->sim_time_;
}