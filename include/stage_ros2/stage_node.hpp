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


// libstage
#include <stage.hh>


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
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "stage_ros2/visibility.h"

// Our node
class StageNode : public rclcpp::Node
{
public:
  STAGE_ROS2_PACKAGE_PUBLIC StageNode(rclcpp::NodeOptions options);

private:
    // A mutex to lock access to fields that are used in message callbacks
    std::mutex msg_lock;

    // The models that we're interested in
    std::vector<Stg::ModelCamera *> cameramodels;
    std::vector<Stg::ModelRanger *> lasermodels;
    std::vector<Stg::ModelPosition *> positionmodels;

    //a structure representing a robot inthe simulator
    struct StageRobot
    {
        //stage related models
        Stg::ModelPosition* positionmodel; //one position
        std::vector<Stg::ModelCamera *> cameramodels; //multiple cameras per position
        std::vector<Stg::ModelRanger *> lasermodels; //multiple rangers per position

        //ros publishers
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub; //one odom
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub; //one ground truth

        std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_pubs; //multiple images
        std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> depth_pubs; //multiple depths
        std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_pubs; //multiple cameras
        std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> laser_pubs; //multiple lasers

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub; //one cmd_vel subscriber
    };

    std::vector<StageRobot const *> robotmodels_;

    // Used to remember initial poses for soft reset
    std::vector<Stg::Pose> initial_poses;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

    bool isDepthCanonical;
    bool use_model_names;
    bool enable_gui_;
    std::string world_file_;

    // A helper function that is executed for each stage model.  We use it
    // to search for models of interest.
    static int ghfunc(Stg::Model* mod, StageNode* node);

    static int s_update(Stg::World* world, StageNode* node);

    // Appends the given robot ID to the given message name.  If omitRobotID
    // is true, an unaltered copy of the name is returned.
    const char *mapName(const char *name, size_t robotID, Stg::Model* mod) const;
    const char *mapName(const char *name, size_t robotID, size_t deviceID, Stg::Model* mod) const;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_;

    // Last time that we received a velocity command
    rclcpp::Time base_last_cmd;
    rclcpp::Duration base_watchdog_timeout;

    // Current simulation time
    rclcpp::Time sim_time;
    
    // Last time we saved global position (for velocity calculation).
    rclcpp::Time base_last_globalpos_time;
    // Last published global pose of each robot
    std::vector<Stg::Pose> base_last_globalpos;

    static geometry_msgs::msg::TransformStamped create_transform_stamped(const tf2::Transform &in, const rclcpp::Time &timestamp, const std::string &frame_id, const std::string &child_frame_id);

    static geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);
public:
    ~StageNode();
    // Constructor
    void init(int argc, char** argv);

    // initialzes and declares ros parameters
    void init_parameter();

    // callback to check changes on the parameters
    void callback_update_parameter();

    // timer to check regulary for parameter changes
    rclcpp::TimerBase::SharedPtr timer_update_parameter_;

    // Subscribe to models of interest.  Currently, we find and subscribe
    // to the first 'laser' model and the first 'position' model.  Returns
    // 0 on success (both models subscribed), -1 otherwise.
    int SubscribeModels();

    // Our callback
    int WorldCallback();
    
    // Do one update of the world.  May pause if the next update time
    // has not yet arrived.
    bool UpdateWorld();

    // Message callback for a MsgBaseVel message, which set velocities.
    void cmdvelReceived(int idx, const geometry_msgs::msg::Twist::SharedPtr msg);

    // Service callback for soft reset
    bool cb_reset_srv(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr);

    // The main simulator object
    Stg::World* world;


};



#endif // STAGE_ROS2_PKG__STAGE_ROS_HPP_
