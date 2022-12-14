#include <chrono>
#include <string>
#include <filesystem>

#include "stage_ros2/stage_node.hpp"
#include <sys/stat.h>
#include <functional>

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

StageNode::StageNode(rclcpp::NodeOptions options)
: Node("stage_ros2", options), count_(0)
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
    double base_watchdog_timeout_sec;
    this->get_parameter("world_file", world_file_);
    this->get_parameter("base_watchdog_timeout", base_watchdog_timeout_sec);
    this->get_parameter("enable_gui", enable_gui_);
    this->get_parameter("use_model_names_", use_model_names_);
    this->base_watchdog_timeout_ = rclcpp::Duration::from_seconds(base_watchdog_timeout_sec);

    if(!std::filesystem::exists(world_file_)){
        RCLCPP_FATAL(this->get_logger(),"The world file %s does not exist.", world_file_.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "callback_update_parameter");
}

void StageNode::init_parameter() {
    
    this->declare_parameter<double>("value_double", value_double);
    this->declare_parameter<double>("base_watchdog_timeout", 0.2);
    this->declare_parameter<bool>("enable_gui", true);
    this->declare_parameter<bool>("use_model_names_", false);
    this->declare_parameter<std::string>("world_file", "cave.world");
    
    /*
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        rcl_interfaces::msg::IntegerRange range;
        range.set__from_value(0).set__to_value(100).set__step(1);
        descriptor.integer_range= {range};
        this->declare_parameter("value_int", 1, descriptor);
    }
    */
        
    callback_update_parameter();
    timer_ = this->create_wall_timer(1000ms, std::bind(&StageNode::callback_update_parameter, this));
}

void StageNode::init(int argc, char** argv){

    // initialize libstage
    Stg::Init( &argc, &argv );

    if(enable_gui_)
        this->stage_ = new Stg::WorldGui(600, 400, "Stage (ROS)");
    else
        this->stage_ = new Stg::World();

    this->stage_->Load(world_file_.c_str());

    this->stage_->AddUpdateCallback((Stg::world_callback_t)s_update, this);
    this->stage_->ForEachDescendant((Stg::model_callback_t)ghfunc, this);
}

void StageNode::start(){

    this->stage_->Start();
    Stg::World::Run();

}

int StageNode::ghfunc(Stg::Model* mod, StageNode* node)
{
  //printf( "inspecting %s, parent\n", mod->Token() );
  return 0;
}

int StageNode::s_update(Stg::World *world, StageNode *node)
{
  return node->callback_world(world);
}

int StageNode::callback_world(Stg::World *world)
{
  if( ! rclcpp::ok() ) {
    RCLCPP_INFO(this->get_logger(), "rclcpp::ok() is false. Quitting." );
    world->QuitAll();
    return 1;
  }
  this->sim_time_ = rclcpp::Time(world->SimTimeNow() * 1e3);
  
  return 0;
}
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(StageNode)