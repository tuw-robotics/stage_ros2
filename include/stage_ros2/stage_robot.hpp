#ifndef STAGE_ROS2_PKG__STAGE_ROBOT_HPP_
#define STAGE_ROS2_PKG__STAGE_ROBOT_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <stage.hh>

// a structure representing a robot inthe simulator
class StageRobot
{
public:
    StageRobot(){};

    // stage related models
    Stg::ModelPosition *positionmodel;            // one position
    //std::vector<Stg::ModelCamera *> cameramodels; // multiple cameras per position
    //std::vector<Stg::ModelRanger *> lasermodels;  // multiple rangers per position

    // ros publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;         // one odom
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub; // one ground truth

    //std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_pubs;       // multiple images
    //std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> depth_pubs;       // multiple depths
    //std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_pubs; // multiple cameras
    //std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> laser_pubs;   // multiple lasers

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub; // one cmd_vel subscriber
};
#endif // STAGE_ROS2_PKG__STAGE_ROBOT_HPP_