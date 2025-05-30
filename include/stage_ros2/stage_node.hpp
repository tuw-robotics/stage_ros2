#ifndef STAGE_ROS2_PKG__STAGE_ROS_HPP_
#define STAGE_ROS2_PKG__STAGE_ROS_HPP_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>
#include <mutex>

// roscpp
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <stage_ros2/transform_broadcaster.h>
#include <stage_ros2/static_transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// libstage
#include <stage.hh>

#include "stage_ros2/visibility.h"



// Our node
class StageNode : public rclcpp::Node
{
public:
  STAGE_ROS2_PACKAGE_PUBLIC StageNode(rclcpp::NodeOptions options);

private:
  // A mutex to lock access to fields that are used in message callbacks
  std::mutex msg_lock;

  // a structure representing a robot inthe simulator
  class Vehicle
  {
public:
    class Ranger
    {
      bool initialized_;
      size_t id_;
      Stg::ModelRanger * model;
      std::shared_ptr<Vehicle> vehicle;
      std::string topic_name;
      std::string frame_base;
      std::string frame_id;
      geometry_msgs::msg::TransformStamped::SharedPtr transform;
      rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub;
      sensor_msgs::msg::LaserScan::SharedPtr msg;
      bool prepare_msg();
      bool prepare_tf();

public:
      Ranger(
        unsigned int id, Stg::ModelRanger * m, std::shared_ptr<Vehicle> & vehicle);
      void init(bool add_id_to_topic);
      unsigned int id() const;
      void publish_msg();
      void publish_tf();
    };
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

private:
    bool initialized_;
    size_t id_;
    Stg::Pose initial_pose_;
    std::string name_;     /// used for the ros publisher
    StageNode * node_;
    Stg::World * world_;
    rclcpp::Time time_last_cmd_received_;
    rclcpp::Time timeout_cmd_;        /// if no command is received befor the vehicle is stopped
    // Last time we saved global position (for velocity calculation).
    rclcpp::Time time_last_pose_update_;

    std::string topic_name_space_;
    std::string frame_name_space_;
    std::string topic_name_cmd_;

    std::string topic_name_tf_;
    std::string topic_name_tf_static_;
    std::string topic_name_odom_;
    std::string topic_name_ground_truth_;
    std::string frame_id_odom_;
    std::string frame_id_world_;
    std::string frame_id_base_link_;
    nav_msgs::msg::Odometry msg_odom_;
    std::shared_ptr<Stg::Pose> global_pose_;

public:
    Vehicle(size_t id, const Stg::Pose & pose, const std::string & name, StageNode * node);

    void soft_reset();
    size_t id() const;
    const std::string & name() const;
    const std::string & name_space() const;
    void init(bool use_topic_prefixes, bool use_one_tf_tree);
    void callback_cmd(const geometry_msgs::msg::Twist::SharedPtr msg);
    void publish_msg();
    void publish_tf();
    void check_watchdog_timeout();
    StageNode *node(){
      return node_;
    }

    // stage related models
    Stg::ModelPosition * positionmodel;               // one position
    std::vector<std::shared_ptr<Ranger>> rangers_;     // multiple rangers per position
    std::vector<std::shared_ptr<Camera>> cameras_;      // multiple cameras per position

    // ros publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;             // one odom
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_ground_truth_;     // one ground truth
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;     // one cmd_vel subscriber

    std::shared_ptr<stage_ros2::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::shared_ptr<stage_ros2::TransformBroadcaster> tf_broadcaster_;
  };

  /// vector to hold the simulated vehicles with ros interfaces
  std::vector<std::shared_ptr<Vehicle>> vehicles_;


  bool isDepthCanonical_;                  /// ROS parameter
  bool enforce_prefixes_;                  /// ROS parameter
  bool one_tf_tree_;                       /// ROS parameter
  bool enable_gui_;                        /// ROS parameter
  bool publish_ground_truth_;              /// ROS parameter
  bool use_static_transformations_;        /// ROS parameter
  std::string world_file_;                 /// ROS parameter
  std::string frame_id_odom_name_;         /// ROS parameter
  std::string frame_id_world_name_;        /// ROS parameter
  std::string frame_id_base_link_name_;    /// ROS parameter

  // TF broadcaster to publish the robot odom
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_stage_;

  // Service to listening on soft reset signals
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset_;

  // publisher for the simulated clock
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

  /// called only ones to init the models and to crate for each model a link to ROS
  static int callback_init_stage_model(Stg::Model * mod, StageNode * node);

  /// called on every simulation interation
  static int callback_update_stage_world(Stg::World * world, StageNode * node);

public:
  ~StageNode();
  // Constructor
  void init(int argc, char ** argv);

  // declares ros parameters
  void declare_parameters();

  // int ros parameters for the startup
  void update_parameters();

  // callback to check changes on the parameters
  void callback_update_parameters();

  // timer to check regulary for parameter changes
  rclcpp::TimerBase::SharedPtr timer_update_parameter_;

  // Subscribe to models of interest.  Currently, we find and subscribe
  // to the first 'laser' model and the first 'position' model.  Returns
  // 0 on success (both models subscribed), -1 otherwise.
  int SubscribeModels();

  // Do one update of the world.  May pause if the next update time
  // has not yet arrived.
  bool UpdateWorld();

  // Service callback for soft reset
  bool cb_reset_srv(const std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr);

  // The main simulator object
  Stg::World * world;

  rclcpp::Duration base_watchdog_timeout_;

  // Current simulation time
  rclcpp::Time sim_time_;

private:
  static geometry_msgs::msg::TransformStamped create_transform_stamped(
    const tf2::Transform & in,
    const rclcpp::Time & timestamp, const std::string & frame_id,
    const std::string & child_frame_id);
  static geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);

};

#endif // STAGE_ROS2_PKG__STAGE_ROS_HPP_
