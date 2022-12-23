#ifndef STAGE_ROS2_PKG__STAGE_ROS_HPP_
#define STAGE_ROS2_PKG__STAGE_ROS_HPP_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>


// libstage
#include <stage.hh>


// roscpp
#include "rclcpp/rclcpp.hpp"
#include <boost/thread/mutex.hpp>
#include <thread>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include "stage_ros2/visibility.h"

#include <std_srvs/srv/empty.hpp>

#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"

#define USAGE "stageros <worldfile>"
#define IMAGE "image"
#define DEPTH "depth"
#define CAMERA_INFO "camera_info"
#define ODOM "odom"
#define BASE_SCAN "base_scan"
#define BASE_POSE_GROUND_TRUTH "base_pose_ground_truth"
#define CMD_VEL "cmd_vel"


// Our node
class StageNode : public rclcpp::Node
{
public:
  STAGE_ROS2_PACKAGE_PUBLIC StageNode(rclcpp::NodeOptions options);

private:
    // A mutex to lock access to fields that are used in message callbacks
    boost::mutex msg_lock;

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
    static void ghfunc(Stg::Model* mod, StageNode* node);

    static bool s_update(Stg::World* world, StageNode* node)
    {
        node->WorldCallback();
        // We return false to indicate that we want to be called again (an
        // odd convention, but that's the way that Stage works).
        return false;
    }

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

public:
    ~StageNode();
    // Constructor; stage itself needs argc/argv.  fname is the .world file
    // that stage should load.
    void init(int argc, char** argv);

    // Subscribe to models of interest.  Currently, we find and subscribe
    // to the first 'laser' model and the first 'position' model.  Returns
    // 0 on success (both models subscribed), -1 otherwise.
    int SubscribeModels();

    // Our callback
    void WorldCallback();
    
    // Do one update of the world.  May pause if the next update time
    // has not yet arrived.
    bool UpdateWorld();

    // Message callback for a MsgBaseVel message, which set velocities.
    void cmdvelReceived(int idx, const geometry_msgs::msg::Twist::SharedPtr msg);

    // Service callback for soft reset
    bool cb_reset_srv(const std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response);

    // The main simulator object
    Stg::World* world;
};



#endif // STAGE_ROS2_PKG__STAGE_ROS_HPP_
