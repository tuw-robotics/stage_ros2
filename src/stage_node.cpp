#include <chrono>

#include "stage_ros2/stage_node.hpp"

using namespace std::chrono_literals;

StageNode::StageNode(rclcpp::NodeOptions options)
: Node("publisher_node", options), count_(0)
{
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


void StageNode::init(int argc, char** argv, bool gui, const char* fname, bool use_model_names){

    struct stat s;
    if(stat(fname, &s) != 0)
    {
        RCLCPP_FATAL(this->get_logger(),"The world file %s does not exist.", fname);
    }

    // initialize libstage
    Stg::Init( &argc, &argv );

    if(gui)
        this->world = new Stg::WorldGui(600, 400, "Stage (ROS)");
    else
        this->world = new Stg::World();

    this->world->Load(fname);
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(StageNode)