#include <stage_ros2/stage_node.hpp>

#include <chrono>
#include <memory>
#include <filesystem>

#define ODOM "odom"
#define FRAME_BASE  "base"
#define FRAME_FOOTPRINT "footprint"
#define FRAME_ODOM "odom"
#define GROUND_TRUTH "ground_truth"
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

void StageNode::Vehicle::init(bool use_model_name)
{
    name_space_ = std::string();
    if (use_model_name) name_space_ = name() + "/";
    frame_id_base_ = name_space_ + FRAME_BASE;
    frame_id_odom_ = name_space_ + FRAME_ODOM;
    frame_id_footprint_ = name_space_ + FRAME_FOOTPRINT;

    topic_name_odom_ = name_space_ + ODOM;
    topic_name_ground_truth_ = name_space_ + GROUND_TRUTH;
    topic_name_cmd_ = name_space_ + CMD_VEL;

    odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>(topic_name_odom_, 10);
    ground_truth_pub = node_->create_publisher<nav_msgs::msg::Odometry>(topic_name_ground_truth_, 10);
    cmdvel_sub = node_->create_subscription<geometry_msgs::msg::Twist>(topic_name_cmd_, 10, std::bind(&StageNode::Vehicle::callback_cmd, this, _1));

    positionmodel->Subscribe();

    for (std::shared_ptr<Ranger> ranger: rangers)
    {
        ranger->init(rangers.size() > 1);
    }

    for (std::shared_ptr<Camera> camera: cameras)
    {
        camera->init(rangers.size() > 1);
    }
}

void StageNode::Vehicle::publish_msg(){
        // Get latest odometry data
        // Translate into ROS message format and publish
        
        msg_odom_.pose.pose.position.x = positionmodel->est_pose.x;
        msg_odom_.pose.pose.position.y = positionmodel->est_pose.y;
        msg_odom_.pose.pose.orientation = createQuaternionMsgFromYaw(positionmodel->est_pose.a);
        Stg::Velocity v = positionmodel->GetVelocity();
        msg_odom_.twist.twist.linear.x = v.x;
        msg_odom_.twist.twist.linear.y = v.y;
        msg_odom_.twist.twist.angular.z = v.a;

        //@todo Publish stall on a separate topic when one becomes available
        // node->odomMsgs[r].stall = node->positionmodels[r]->Stall();
        //
        msg_odom_.header.frame_id = frame_id_odom_;
        msg_odom_.header.stamp = node_->sim_time_;

        odom_pub->publish(msg_odom_);

        // Also publish the ground truth pose and velocity
        Stg::Pose gpose = positionmodel->GetGlobalPose();
        tf2::Quaternion q_gpose;
        q_gpose.setRPY(0.0, 0.0, gpose.a);
        tf2::Transform gt(q_gpose, tf2::Vector3(gpose.x, gpose.y, 0.0));
        // Velocity is 0 by default and will be set only if there is previous pose and time delta>0
        Stg::Velocity gvel(0, 0, 0, 0);
        if (global_pose_)
        {
            double dT = (node_->sim_time_ - node_->base_last_globalpos_time_).seconds();
            if (dT > 0)
                gvel = Stg::Velocity(
                    (gpose.x - global_pose_->x) / dT,
                    (gpose.y - global_pose_->y) / dT,
                    (gpose.z - global_pose_->z) / dT,
                    Stg::normalize(gpose.a - global_pose_->a) / dT);
            *global_pose_ = gpose;
        }
        else {
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

        ground_truth_msg.header.frame_id = frame_id_odom_;
        ground_truth_msg.header.stamp = node_->sim_time_;

        ground_truth_pub->publish(ground_truth_msg);

}
void StageNode::Vehicle::publish_tf(){

        // broadcast odometry transform
        tf2::Quaternion quaternion = tf2::Quaternion(
            msg_odom_.pose.pose.orientation.x,
            msg_odom_.pose.pose.orientation.y,
            msg_odom_.pose.pose.orientation.z,
            msg_odom_.pose.pose.orientation.w);
        tf2::Transform transform(quaternion, tf2::Vector3(msg_odom_.pose.pose.position.x, msg_odom_.pose.pose.position.y, 0.0));
        node_->tf_->sendTransform(create_transform_stamped(transform, node_->sim_time_,
                                                          frame_id_odom_,
                                                          frame_id_footprint_));

        // the position of the robot
        node_->tf_->sendTransform(create_transform_stamped(tf2::Transform::getIdentity(),
                                                          node_->sim_time_,
                                                          frame_id_footprint_,
                                                          frame_id_base_));
}

void StageNode::Vehicle::callback_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::scoped_lock lock(node_->msg_lock);
    this->positionmodel->SetSpeed(msg->linear.x,
                                  msg->linear.y,
                                  msg->angular.z);
    node_->base_last_cmd_ = node_->sim_time_;
}