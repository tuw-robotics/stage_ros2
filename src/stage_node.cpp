#include <stage_ros2/stage_node.hpp>

#include <chrono>
#include <memory>
#include <filesystem>

StageNode::StageNode(rclcpp::NodeOptions options)
    : Node("stage_ros2", options), base_watchdog_timeout_(0, 0)
{
    tf_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    init_parameter();
}

StageNode::~StageNode()
{
}

void StageNode::init_parameter()
{
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    auto param_desc_enable_gui = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc_enable_gui.description = "Enable GUI!";
    this->declare_parameter<bool>("enable_gui", true, param_desc_enable_gui);

    auto param_desc_model_names = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc_model_names.description = "USE model name as name space prefix! True on more than one vehicle!";
    this->declare_parameter<bool>("use_model_names", false, param_desc_model_names);

    auto param_desc_watchdog_timeout = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc_watchdog_timeout.description = "timeout after which a vehicle stopps if no command is received!";
    this->declare_parameter<double>("base_watchdog_timeout", 0.2, param_desc_watchdog_timeout);

    auto param_desc_is_depth_canonical = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc_is_depth_canonical.description = "USE depth canonical!";
    this->declare_parameter<bool>("is_depth_canonical", true, param_desc_is_depth_canonical);

    auto param_desc_publish_ground_truth = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc_publish_ground_truth.description = "publishes on true a ground truth tf!";
    this->declare_parameter<bool>("publish_ground_truth", true, param_desc_publish_ground_truth);

    auto param_desc_world_file = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc_world_file.description = "USE model names!";
    this->declare_parameter<std::string>("world_file", "cave.world", param_desc_world_file);

    callback_update_parameter();

    using namespace std::chrono_literals;
    timer_update_parameter_ = this->create_wall_timer(1000ms, std::bind(&StageNode::callback_update_parameter, this));
}

void StageNode::callback_update_parameter()
{
    double base_watchdog_timeout_sec{0.2};
    this->get_parameter("base_watchdog_timeout", base_watchdog_timeout_sec);
    this->base_watchdog_timeout_ = rclcpp::Duration::from_seconds(base_watchdog_timeout_sec);

    this->get_parameter("publish_ground_truth", this->publish_ground_truth_);
    RCLCPP_INFO(this->get_logger(), "callback_update_parameter");
}
/**
 * Is called only ones after the simulation starts with each model
 * The function fills the vehicle vector with pointers to the stage models
 * @param mod stage model
 * @param node pointer to this class
*/
int StageNode::ghfunc(Stg::Model *mod, StageNode *node)
{
    if (dynamic_cast<Stg::ModelPosition *>(mod))
    {
        Stg::ModelPosition *position = dynamic_cast<Stg::ModelPosition *>(mod);
        RCLCPP_INFO(node->get_logger(), "New Vehicle \"%s\"", mod->TokenStr().c_str());
        auto vehicle = std::make_shared<Vehicle>(node->vehicles_.size(), position->GetGlobalPose(), mod->TokenStr(), node);
        node->vehicles_.push_back(vehicle);
        vehicle->positionmodel = position;
    }

    if (dynamic_cast<Stg::ModelRanger *>(mod))
    {
        Stg::ModelPosition *parent = dynamic_cast<Stg::ModelPosition *>(mod->Parent());
        for (std::shared_ptr<Vehicle> vehcile: node->vehicles_){
            if (parent == vehcile->positionmodel){
                auto ranger = std::make_shared<Vehicle::Ranger>(vehcile->rangers.size()+1, dynamic_cast<Stg::ModelRanger *>(mod), vehcile, node);
                vehcile->rangers.push_back(ranger);
            }
        }
    }
    if (dynamic_cast<Stg::ModelCamera *>(mod))
    {
        Stg::ModelPosition *parent = dynamic_cast<Stg::ModelPosition *>(mod->Parent());
        for (std::shared_ptr<Vehicle> vehcile: node->vehicles_){
            if (parent == vehcile->positionmodel){
                auto camera = std::make_shared<Vehicle::Camera>(vehcile->cameras.size()+1, dynamic_cast<Stg::ModelCamera *>(mod), vehcile, node);
                vehcile->cameras.push_back(camera);
            }
        }
    }
    return 0;
}

int StageNode::s_update(Stg::World *world, StageNode *node)
{
    // We return false to indicate that we want to be called again (an
    // odd convention, but that's the way that Stage works).
    if (!rclcpp::ok())
    {
        RCLCPP_INFO(node->get_logger(), "rclcpp::ok() is false. Quitting.");
        node->world->QuitAll();
        return 1;
    }

    std::scoped_lock lock(node->msg_lock);

    node->sim_time_ = rclcpp::Time(world->SimTimeNow() * 1e3);
    // We're not allowed to publish clock==0, because it used as a special
    // value in parts of ROS, #4027.
    if (int(node->sim_time_.nanoseconds()) == 0)
    {
        RCLCPP_DEBUG(node->get_logger(), "Skipping initial simulation step, to avoid publishing clock==0");
        return 0;
    }

    // TODO make this only affect one robot if necessary
    if ((node->base_watchdog_timeout_.nanoseconds() > 0) &&
        ((node->sim_time_ - node->base_last_cmd_) >= node->base_watchdog_timeout_))
    {
        for (size_t r = 0; r < node->vehicles_.size(); r++)
            node->vehicles_[r]->positionmodel->SetSpeed(0.0, 0.0, 0.0);
    }

    // loop on the robot models
    for (size_t r = 0; r < node->vehicles_.size(); ++r)
    {
        auto robotmodel = node->vehicles_[r];
        robotmodel->publish_msg();
        robotmodel->publish_tf();

        // loop on the ranger devices for the current robot
        for (auto ranger: robotmodel->rangers)
        {
            ranger->publish_msg();
            ranger->publish_tf();
        }


        // loop on the camera devices for the current robot
        for (auto camera: robotmodel->cameras)
        {
             camera->publish_msg();
             camera->publish_tf();
        }
    }

    node->base_last_globalpos_time_ = node->sim_time_;
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = node->sim_time_;
    node->clock_pub_->publish(clock_msg);
    return 0;
}

bool StageNode::cb_reset_srv(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr)
{
    RCLCPP_INFO(this->get_logger(), "Resetting stage!");
    for (auto vehicle: this->vehicles_) vehicle->soft_reset();
    return true;
}

void StageNode::init(int argc, char **argv)
{
    double base_watchdog_timeout_sec{5.0};
    this->get_parameter("enable_gui", this->enable_gui_);
    this->get_parameter("use_model_names", this->use_model_names);
    this->get_parameter("base_watchdog_timeout", base_watchdog_timeout_sec);
    this->base_watchdog_timeout_ = rclcpp::Duration::from_seconds(base_watchdog_timeout_sec);
    this->get_parameter("is_depth_canonical", this->isDepthCanonical_);
    this->get_parameter("world_file", this->world_file_);
    this->get_parameter("publish_ground_truth", this->publish_ground_truth_);

    this->sim_time_ = rclcpp::Time(0, 0);
    this->base_last_cmd_ = rclcpp::Time(0, 0);

    if (!std::filesystem::exists(world_file_))
    {
        RCLCPP_FATAL(this->get_logger(), "The world file %s does not exist.", this->world_file_.c_str());
        exit(0);
    }

    // initialize libstage
    Stg::Init(&argc, &argv);

    if (this->enable_gui_)
        this->world = new Stg::WorldGui(600, 400, "Stage (ROS)");
    else
        this->world = new Stg::World();

    this->world->Load(world_file_.c_str());

    // todo: reverse the order of these next lines? try it .

    this->world->AddUpdateCallback((Stg::world_callback_t)s_update, this);

    // inspect every model to locate the things we care about
    this->world->ForEachDescendant((Stg::model_callback_t)ghfunc, this);
}

// Subscribe to models of interest.  Currently, we find and subscribe
// to the first 'laser' model and the first 'position' model.  Returns
// 0 on success (both models subscribed), -1 otherwise.
//
// Eventually, we should provide a general way to map stage models onto ROS
// topics, similar to Player .cfg files.
int StageNode::SubscribeModels()
{
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    for (std::shared_ptr<Vehicle> vehicle: this->vehicles_)
    {
        // init topics and use the stage models names if there are more than one vehicle in the world
        vehicle->init(this->use_model_names || (vehicles_.size() > 1));
    }

    // create the clock publisher
    clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

    // advertising reset service
    reset_srv_ = this->create_service<std_srvs::srv::Empty>("reset_positions", [this](const std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response)
                                                            { this->cb_reset_srv(request, response); });

    return (0);
}

bool StageNode::UpdateWorld()
{
    return this->world->UpdateAll();
}

// helper functions
geometry_msgs::msg::TransformStamped StageNode::create_transform_stamped(const tf2::Transform &in, const rclcpp::Time &timestamp, const std::string &frame_id, const std::string &child_frame_id)
{
    geometry_msgs::msg::TransformStamped out;
    out.header.stamp = timestamp;
    out.header.frame_id = frame_id;
    out.child_frame_id = child_frame_id;
    out.transform.translation.x = in.getOrigin().getX();
    out.transform.translation.y = in.getOrigin().getY();
    out.transform.translation.z = in.getOrigin().getZ();
    out.transform.rotation.w = in.getRotation().getW();
    out.transform.rotation.x = in.getRotation().getX();
    out.transform.rotation.y = in.getRotation().getY();
    out.transform.rotation.z = in.getRotation().getZ();
    return out;
}

geometry_msgs::msg::Quaternion StageNode::createQuaternionMsgFromYaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}