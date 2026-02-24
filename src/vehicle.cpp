#include "stage_ros2/vehicle.hpp"
#include "stage_ros2/stage_node.hpp"

#include <memory>

namespace stage_ros2 {

#define TOPIC_TF "tf"
#define TOPIC_TF_STATIC "tf_static"
#define TOPIC_ODOM "odom"
#define TOPIC_GROUND_TRUTH "ground_truth"
#define TOPIC_CMD_VEL "cmd_vel"
#define TOPIC_DRIVE "drive"

using std::placeholders::_1;

Vehicle::Vehicle(
    size_t id, const Stg::Pose &pose, const std::string &name,
    StageNode *node)
    : initialized_(false), id_(id), initial_pose_(pose), name_(name), node_(node)
{
}

size_t Vehicle::id() const
{
  return id_;
}
void Vehicle::soft_reset()
{
  positionmodel->SetPose(this->initial_pose_);
  positionmodel->SetStall(false);
}

const std::string &Vehicle::name() const
{
  return name_;
}

void Vehicle::init(bool use_topic_prefixes, bool use_one_tf_tree)
{
  if (initialized_)
    return;

  time_last_pose_update_ = rclcpp::Time(0, 0);
  time_last_cmd_received_ = rclcpp::Time(0, 0);
  timeout_cmd_ = rclcpp::Time(0, 0);

  topic_name_space_ = std::string();
  frame_name_space_ = std::string();
  if (use_topic_prefixes == true)
  {
    topic_name_space_ = name() + "/";
  }
  if (use_one_tf_tree)
  {
    frame_name_space_ = name() + "/";
    topic_name_tf_ = std::string("/") + TOPIC_TF;
    topic_name_tf_static_ =  std::string("/") + TOPIC_TF_STATIC;
  } else {
    topic_name_tf_ = topic_name_space_ + TOPIC_TF;
    topic_name_tf_static_ = topic_name_space_ + TOPIC_TF_STATIC;
  }

  frame_id_base_link_ = frame_name_space_ + node_->frame_id_base_link_name_;
  frame_id_odom_ = frame_name_space_ + node_->frame_id_odom_name_;
  frame_id_world_ = frame_name_space_ + node_->frame_id_world_name_;

  topic_name_odom_ = topic_name_space_ + TOPIC_ODOM;
  topic_name_ground_truth_ = topic_name_space_ + TOPIC_GROUND_TRUTH;
  topic_name_cmd_ = topic_name_space_ + TOPIC_CMD_VEL;
  topic_name_drive_ = topic_name_space_ + TOPIC_DRIVE;

  tf_static_broadcaster_ = std::make_shared<stage_ros2::StaticTransformBroadcaster>(node_, topic_name_tf_static_.c_str());
  tf_broadcaster_ = std::make_shared<stage_ros2::TransformBroadcaster>(node_, topic_name_tf_.c_str());

  pub_odom_ = node_->create_publisher<nav_msgs::msg::Odometry>(topic_name_odom_, 10);
  pub_ground_truth_ =
      node_->create_publisher<nav_msgs::msg::Odometry>(topic_name_ground_truth_, 10);

  if(node_->use_ackermann_){
      if(node_->use_stamped_velocity_){
        sub_drive_stamped_ =
          node_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
              topic_name_drive_, 10,
              std::bind(&Vehicle::callback_drive_stamped, this, _1));
        RCLCPP_INFO(node_->get_logger(), "%s is using stamped Ackermann velocity commands.", name().c_str());
      } else {
        sub_drive_ =
            node_->create_subscription<ackermann_msgs::msg::AckermannDrive>(
                topic_name_drive_, 10,
                std::bind(&Vehicle::callback_drive, this, _1));
        RCLCPP_INFO(node_->get_logger(), "%s is using unstamped Ackermann velocity commands.", name().c_str());
      }
  } else {
      if(node_->use_stamped_velocity_){
        sub_cmd_stamped_ =
          node_->create_subscription<geometry_msgs::msg::TwistStamped>(
              topic_name_cmd_, 10,
              std::bind(&Vehicle::callback_cmd_stamped, this, _1));
        RCLCPP_INFO(node_->get_logger(), "%s is using stamped velocity commands.", name().c_str());
      } else {
        sub_cmd_ =
            node_->create_subscription<geometry_msgs::msg::Twist>(
                topic_name_cmd_, 10,
                std::bind(&Vehicle::callback_cmd, this, _1));
        RCLCPP_INFO(node_->get_logger(), "%s is using unstamped velocity commands.", name().c_str());
      }
  }
  positionmodel->Subscribe();

  for (auto &ranger : rangers_)
  {
    ranger->init(rangers_.size() > 1);
  }

  for (auto &detector : fiducial_detectors_)
  {
    detector->init(fiducial_detectors_.size() > 1);
  }

  for (auto &camera : cameras_)
  {
    camera->init(cameras_.size() > 1);
  }
  initialized_ = true;
}

void Vehicle::publish_msg()
{
  // Guard
  if (!initialized_)
    return;

  // Get latest odometry data
  // Translate into ROS message format and publish
  msg_odom_.pose.pose.position.x = positionmodel->est_pose.x;
  msg_odom_.pose.pose.position.y = positionmodel->est_pose.y;
  msg_odom_.pose.pose.orientation = StageNode::createQuaternionMsgFromYaw(positionmodel->est_pose.a);
  Stg::Velocity v = positionmodel->GetVelocity();
  msg_odom_.twist.twist.linear.x = v.x;
  msg_odom_.twist.twist.linear.y = v.y;
  msg_odom_.twist.twist.angular.z = v.a;
  msg_odom_.header.frame_id = frame_id_odom_;
  msg_odom_.header.stamp = node_->sim_time_;
  msg_odom_.child_frame_id = frame_id_base_link_;

  pub_odom_->publish(msg_odom_);

  // Also publish the ground truth pose and velocity
  Stg::Pose gpose = positionmodel->GetGlobalPose();
  tf2::Quaternion q_gpose;
  q_gpose.setRPY(0.0, 0.0, gpose.a);
  tf2::Transform gt(q_gpose, tf2::Vector3(gpose.x, gpose.y, 0.0));
  // Velocity is 0 by default and will be set only if there is previous pose and time delta>0
  // @ToDo using the positionmodel->GetVelocity() a self computed delta
  Stg::Velocity gvel(0, 0, 0, 0);
  if (global_pose_)
  {
    double dT = (node_->sim_time_ - time_last_pose_update_).seconds();
    if (dT > 0)
    {
      gvel = Stg::Velocity(
          (gpose.x - global_pose_->x) / dT,
          (gpose.y - global_pose_->y) / dT,
          (gpose.z - global_pose_->z) / dT,
          Stg::normalize(gpose.a - global_pose_->a) / dT);
    }
    *global_pose_ = gpose;
  }
  else
  {
    // There are no previous readings, adding current pose...
    global_pose_ = std::make_shared<Stg::Pose>(gpose);
  }
  nav_msgs::msg::Odometry ground_truth_msg;
  ground_truth_msg.pose.pose.position.x = gt.getOrigin().x();
  ground_truth_msg.pose.pose.position.y = gt.getOrigin().y();
  ground_truth_msg.pose.pose.position.z = gt.getOrigin().z();
  ground_truth_msg.pose.pose.orientation.x = gt.getRotation().x();
  ground_truth_msg.pose.pose.orientation.y = gt.getRotation().y();
  ground_truth_msg.pose.pose.orientation.z = gt.getRotation().z();
  ground_truth_msg.pose.pose.orientation.w = gt.getRotation().w();
  ground_truth_msg.twist.twist.linear.x = gvel.x;
  ground_truth_msg.twist.twist.linear.y = gvel.y;
  ground_truth_msg.twist.twist.linear.z = gvel.z;
  ground_truth_msg.twist.twist.angular.z = gvel.a;

  ground_truth_msg.header.frame_id = frame_id_world_;
  ground_truth_msg.header.stamp = node_->sim_time_;

  pub_ground_truth_->publish(ground_truth_msg);
  time_last_pose_update_ = node_->sim_time_;
}
void Vehicle::publish_tf()
{

  // broadcast odometry transform
  tf2::Quaternion quaternion = tf2::Quaternion(
      msg_odom_.pose.pose.orientation.x,
      msg_odom_.pose.pose.orientation.y,
      msg_odom_.pose.pose.orientation.z,
      msg_odom_.pose.pose.orientation.w);
  tf2::Transform transform(quaternion,
                           tf2::Vector3(msg_odom_.pose.pose.position.x, msg_odom_.pose.pose.position.y, 0.0));
  tf_broadcaster_->sendTransform(
      StageNode::create_transform_stamped(
          transform, node_->sim_time_,
          frame_id_odom_,
          frame_id_base_link_));
}

void Vehicle::check_watchdog_timeout()
{

  if ((timeout_cmd_ != rclcpp::Time(0, 0)) && (node_->sim_time_ > timeout_cmd_))
  {
    Stg::Velocity v = positionmodel->GetVelocity();
    // stopping makes only sense if the vehicle drives
    if (!positionmodel->GetVelocity().IsZero())
    {
      this->positionmodel->SetSpeed(0.0, 0.0, 0.0);
      RCLCPP_INFO(node_->get_logger(), "watchdog timeout on %s", name().c_str());
    }
  }
}
void Vehicle::callback_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::scoped_lock lock(node_->msg_lock);
  this->positionmodel->SetSpeed(
      msg->linear.x,
      msg->linear.y,
      msg->angular.z);
  time_last_cmd_received_ = node_->sim_time_;
  timeout_cmd_ = time_last_cmd_received_ + node_->base_watchdog_timeout_;
}

void Vehicle::callback_cmd_stamped(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::scoped_lock lock(node_->msg_lock);
  this->positionmodel->SetSpeed(
      msg->twist.linear.x,
      msg->twist.linear.y,
      msg->twist.angular.z);
  time_last_cmd_received_ = node_->sim_time_;
  timeout_cmd_ = time_last_cmd_received_ + node_->base_watchdog_timeout_;
}

void Vehicle::callback_drive(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg)
{
  std::scoped_lock lock(node_->msg_lock);
  this->positionmodel->SetSpeed(
      msg->speed,
      0.0,
      msg->steering_angle);
  time_last_cmd_received_ = node_->sim_time_;
  timeout_cmd_ = time_last_cmd_received_ + node_->base_watchdog_timeout_;
}

void Vehicle::callback_drive_stamped(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
  std::scoped_lock lock(node_->msg_lock);
  this->positionmodel->SetSpeed(
      msg->drive.speed,
      0.0,
      msg->drive.steering_angle);
  time_last_cmd_received_ = node_->sim_time_;
  timeout_cmd_ = time_last_cmd_received_ + node_->base_watchdog_timeout_;
}

}  // namespace stage_ros2