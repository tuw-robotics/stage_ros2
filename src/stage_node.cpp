#include <chrono>
#include <string>

#include "stage_ros2/stage_node.hpp"
#include <sys/stat.h>

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

StageNode::StageNode(rclcpp::NodeOptions options)
: Node("publisher_node", options), count_(0)
{
  init_parameter();
  publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = create_wall_timer(
    500ms, std::bind(&StageNode::on_timer, this));
}

void StageNode::on_timer()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publisher: '%s'", message.data.c_str());
  publisher_->publish(message);
}


void StageNode::callback_update_parameter()
{
    
    this->get_parameter("accelleration", value_double);
    this->get_parameter("value_int", value_int);
    
    RCLCPP_INFO(this->get_logger(), "callback_update_parameter");
}

void StageNode::init_parameter() {
    
    this->declare_parameter<double>("value_double", value_double);
    
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        rcl_interfaces::msg::IntegerRange range;
        range.set__from_value(0).set__to_value(100).set__step(1);
        descriptor.integer_range= {range};
        this->declare_parameter("value_int", 1, descriptor);
    }
    
    
    callback_update_parameter();
    timer_ = this->create_wall_timer(1000ms, std::bind(&StageNode::callback_update_parameter, this));
}

void StageNode::init(int argc, char** argv, bool gui, const char* fname, bool use_model_names){

    std::string file = "/home/markus/projects/ros2/mobile_robotics/ws00/src/stage_ros2/world/cave.world";

    // initialize libstage
    Stg::Init( &argc, &argv );

    if(gui)
        this->world = new Stg::WorldGui(600, 400, "Stage (ROS)");
    else
        this->world = new Stg::World();

    //this->world->Load(file.c_str());
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(StageNode)