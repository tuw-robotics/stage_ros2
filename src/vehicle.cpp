#include <stage_ros2/stage_node.hpp>

#include <chrono>
#include <memory>
#include <filesystem>

#define IMAGE "image"
#define DEPTH "depth"
#define CAMERA_INFO "camera_info"
#define ODOM "odom"
#define TOPIC_LASER "base_scan"
#define FRAME_LASER "laser"
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
    name_space_ = std::string();
    if (use_model_name) name_space_ = name() + "/";


    odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>(name_space_ + ODOM, 10);
    ground_truth_pub = node_->create_publisher<nav_msgs::msg::Odometry>(name_space_ + BASE_POSE_GROUND_TRUTH, 10);
    cmdvel_sub = node_->create_subscription<geometry_msgs::msg::Twist>(name_space_ + CMD_VEL, 10, std::bind(&StageNode::Vehicle::callback_cmd, this, _1));

    positionmodel->Subscribe();

    for (std::shared_ptr<Ranger> ranger: rangers)
    {
        ranger->model->Subscribe();
        if (rangers.size() == 1)
        {
            ranger->topic_name = name_space_ + TOPIC_LASER;
            ranger->frame_id   = name_space_ + FRAME_LASER;
        }
        else
        {
            ranger->topic_name = name_space_ + TOPIC_LASER + std::to_string(ranger->id);
            ranger->frame_id   = name_space_ + FRAME_LASER + std::to_string(ranger->id);
        }
        ranger->pub = node_->create_publisher<sensor_msgs::msg::LaserScan>(ranger->topic_name, 10);
    }

    for (std::shared_ptr<Camera> camera: cameras)
    {
        camera->model->Subscribe();
        if (cameras.size() == 1)
        {
            camera->topic_name_image = name_space_ + IMAGE;
            camera->topic_name_camera_info = name_space_ + CAMERA_INFO;
            camera->topic_name_depth = name_space_ + DEPTH;
        }
        else
        {
            camera->topic_name_image = name_space_ + IMAGE + std::to_string(camera->id);
            camera->topic_name_camera_info = name_space_ + CAMERA_INFO + std::to_string(camera->id);
            camera->topic_name_depth = name_space_ + DEPTH + std::to_string(camera->id);
        }
        camera->pub_image = node_->create_publisher<sensor_msgs::msg::Image>(camera->topic_name_image, 10);
        camera->pub_camera = node_->create_publisher<sensor_msgs::msg::CameraInfo>(camera->topic_name_camera_info, 10);
        camera->pub_depth = node_->create_publisher<sensor_msgs::msg::Image>(camera->topic_name_depth, 10);
    }
}

void StageNode::Vehicle::publish_rangers(){

}

void StageNode::Vehicle::callback_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::scoped_lock lock(node_->msg_lock);
    this->positionmodel->SetSpeed(msg->linear.x,
                                  msg->linear.y,
                                  msg->angular.z);
    node_->base_last_cmd_ = node_->sim_time_;
}