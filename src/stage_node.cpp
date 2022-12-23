
#include "stage_ros2/stage_node.hpp"

// helper functions
geometry_msgs::msg::TransformStamped create_transform_stamped(const tf2::Transform &in, const rclcpp::Time &timestamp, const std::string &frame_id, const std::string &child_frame_id)
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

auto createQuaternionMsgFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

// since stageros is single-threaded, this is OK. revisit if that changes!
const char *
StageNode::mapName(const char *name, size_t robotID, Stg::Model* mod) const
{
    //ROS_INFO("Robot %lu: Device %s", robotID, name);
    bool umn = this->use_model_names;

    if ((positionmodels.size() > 1 ) || umn)
    {
        static char buf[100];
        std::size_t found = std::string(((Stg::Ancestor *) mod)->Token()).find(":");

        if ((found==std::string::npos) && umn)
        {
            snprintf(buf, sizeof(buf), "/%s/%s", ((Stg::Ancestor *) mod)->Token(), name);
        }
        else
        {
            snprintf(buf, sizeof(buf), "/robot_%u/%s", (unsigned int)robotID, name);
        }

        return buf;
    }
    else
        return name;
}

const char *
StageNode::mapName(const char *name, size_t robotID, size_t deviceID, Stg::Model* mod) const
{
    //ROS_INFO("Robot %lu: Device %s:%lu", robotID, name, deviceID);
    bool umn = this->use_model_names;

    if ((positionmodels.size() > 1 ) || umn)
    {
        static char buf[100];
        std::size_t found = std::string(((Stg::Ancestor *) mod)->Token()).find(":");

        if ((found==std::string::npos) && umn)
        {
            snprintf(buf, sizeof(buf), "/%s/%s_%u", ((Stg::Ancestor *) mod)->Token(), name, (unsigned int)deviceID);
        }
        else
        {
            snprintf(buf, sizeof(buf), "/robot_%u/%s_%u", (unsigned int)robotID, name, (unsigned int)deviceID);
        }

        return buf;
    }
    else
    {
        static char buf[100];
        snprintf(buf, sizeof(buf), "/%s_%u", name, (unsigned int)deviceID);
        return buf;
    }
}

void
StageNode::ghfunc(Stg::Model* mod, StageNode* node)
{
  //printf( "inspecting %s, parent\n", mod->Token() );

  if (dynamic_cast<Stg::ModelRanger *>(mod)) {
     node->lasermodels.push_back(dynamic_cast<Stg::ModelRanger *>(mod));
  }
  if (dynamic_cast<Stg::ModelPosition *>(mod)) {
     Stg::ModelPosition * p = dynamic_cast<Stg::ModelPosition *>(mod);
      // remember initial poses
      node->positionmodels.push_back(p);
      node->initial_poses.push_back(p->GetGlobalPose());
    }
  if (dynamic_cast<Stg::ModelCamera *>(mod)) {
     node->cameramodels.push_back(dynamic_cast<Stg::ModelCamera *>(mod));
  }
}




bool
StageNode::cb_reset_srv(const std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response)
{
  RCLCPP_INFO(this->n_->get_logger(), "Resetting stage!");
  for (size_t r = 0; r < this->positionmodels.size(); r++) {
    this->positionmodels[r]->SetPose(this->initial_poses[r]);
    this->positionmodels[r]->SetStall(false);
  }
  return true;
}



void
StageNode::cmdvelReceived(int idx, const geometry_msgs::msg::Twist::SharedPtr msg)
{
    boost::mutex::scoped_lock lock(msg_lock);
    this->positionmodels[idx]->SetSpeed(msg->linear.x,
                                        msg->linear.y,
                                        msg->angular.z);
    this->base_last_cmd = this->sim_time;
}

StageNode::StageNode(int argc, char** argv, bool gui, const char* fname, bool use_model_names)
: base_watchdog_timeout(0,0)
{
    this->use_model_names = use_model_names;
    this->sim_time = rclcpp::Time(0, 0);
    this->base_last_cmd = rclcpp::Time(0, 0);
    double t;
    rclcpp::Node::SharedPtr localn = rclcpp::Node::make_shared("_");
    if(!localn->get_parameter("base_watchdog_timeout", t))
        t = 0.2;
    this->base_watchdog_timeout = rclcpp::Duration::from_seconds(t);

    if(!localn->get_parameter("is_depth_canonical", isDepthCanonical))
        isDepthCanonical = true;

    // We'll check the existence of the world file, because libstage doesn't
    // expose its failure to open it.  Could go further with checks (e.g., is
    // it readable by this user).
    struct stat s;
    if(stat(fname, &s) != 0)
    {
        RCLCPP_FATAL(this->n_->get_logger(),"The world file %s does not exist.", fname);
    }

    // initialize libstage
    Stg::Init( &argc, &argv );

    if(gui)
        this->world = new Stg::WorldGui(600, 400, "Stage (ROS)");
    else
        this->world = new Stg::World();

    this->world->Load(fname);

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
int
StageNode::SubscribeModels()
{
    n_->set_parameter(rclcpp::Parameter("use_sim_time", true));

    for (size_t r = 0; r < this->positionmodels.size(); r++)
    {
        StageRobot* new_robot = new StageRobot;
        new_robot->positionmodel = this->positionmodels[r];
        new_robot->positionmodel->Subscribe();

	RCLCPP_INFO(this->n_->get_logger(), "Subscribed to Stage position model \"%s\"", this->positionmodels[r]->Token() ); 
		      
        for (size_t s = 0; s < this->lasermodels.size(); s++)
        {
	  if (this->lasermodels[s] and this->lasermodels[s]->Parent() == new_robot->positionmodel)
            {
                new_robot->lasermodels.push_back(this->lasermodels[s]);
                this->lasermodels[s]->Subscribe();
	      RCLCPP_INFO(this->n_->get_logger(), "subscribed to Stage ranger \"%s\"", this->lasermodels[s]->Token() ); 
            }
        }

        for (size_t s = 0; s < this->cameramodels.size(); s++)
        {
            if (this->cameramodels[s] and this->cameramodels[s]->Parent() == new_robot->positionmodel)
            {
                new_robot->cameramodels.push_back(this->cameramodels[s]);
                this->cameramodels[s]->Subscribe();

		RCLCPP_INFO(this->n_->get_logger(), "subscribed to Stage camera model \"%s\"", this->cameramodels[s]->Token() ); 
            }
        }

	// TODO - print the topic names nicely as well
        RCLCPP_INFO(this->n_->get_logger(), "Robot %s provided %lu rangers and %lu cameras",
		 new_robot->positionmodel->Token(),
		 new_robot->lasermodels.size(),
		 new_robot->cameramodels.size() );

        new_robot->odom_pub = n_->create_publisher<nav_msgs::msg::Odometry>(mapName(ODOM, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10);
        new_robot->ground_truth_pub = n_->create_publisher<nav_msgs::msg::Odometry>(mapName(BASE_POSE_GROUND_TRUTH, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10);
        new_robot->cmdvel_sub = n_->create_subscription<geometry_msgs::msg::Twist>(mapName(CMD_VEL, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10, [this, r](const geometry_msgs::msg::Twist::SharedPtr msg) {this->cmdvelReceived(r, msg);});

        for (size_t s = 0;  s < new_robot->lasermodels.size(); ++s)
        {
            if (new_robot->lasermodels.size() == 1)
                new_robot->laser_pubs.push_back(n_->create_publisher<sensor_msgs::msg::LaserScan>(mapName(BASE_SCAN, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
            else
                new_robot->laser_pubs.push_back(n_->create_publisher<sensor_msgs::msg::LaserScan>(mapName(BASE_SCAN, r, s, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));

        }

        for (size_t s = 0;  s < new_robot->cameramodels.size(); ++s)
        {
            if (new_robot->cameramodels.size() == 1)
            {
                new_robot->image_pubs.push_back(n_->create_publisher<sensor_msgs::msg::Image>(mapName(IMAGE, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
                new_robot->depth_pubs.push_back(n_->create_publisher<sensor_msgs::msg::Image>(mapName(DEPTH, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
                new_robot->camera_pubs.push_back(n_->create_publisher<sensor_msgs::msg::CameraInfo>(mapName(CAMERA_INFO, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
            }
            else
            {
                new_robot->image_pubs.push_back(n_->create_publisher<sensor_msgs::msg::Image>(mapName(IMAGE, r, s, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
                new_robot->depth_pubs.push_back(n_->create_publisher<sensor_msgs::msg::Image>(mapName(DEPTH, r, s, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
                new_robot->camera_pubs.push_back(n_->create_publisher<sensor_msgs::msg::CameraInfo>(mapName(CAMERA_INFO, r, s, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
            }
        }

        this->robotmodels_.push_back(new_robot);
    }
    clock_pub_ = n_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

    // advertising reset service
    reset_srv_ = n_->create_service<std_srvs::srv::Empty>("reset_positions", [this](const std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response){this->cb_reset_srv(request, response);});

    return(0);
}

StageNode::~StageNode()
{    
    for (std::vector<StageRobot const*>::iterator r = this->robotmodels_.begin(); r != this->robotmodels_.end(); ++r)
        delete *r;
}

bool
StageNode::UpdateWorld()
{
    return this->world->UpdateAll();
}

void
StageNode::WorldCallback()
{
  if( ! rclcpp::ok() ) {
    RCLCPP_INFO(this->n_->get_logger(), "rclcpp::ok() is false. Quitting." );
    this->world->QuitAll();
    return;
  }
  
    boost::mutex::scoped_lock lock(msg_lock);

    this->sim_time = rclcpp::Time(world->SimTimeNow() * 1e3);
    // We're not allowed to publish clock==0, because it used as a special
    // value in parts of ROS, #4027.
    if(int(this->sim_time.nanoseconds()) == 0)
    {
        RCLCPP_DEBUG(this->n_->get_logger(),"Skipping initial simulation step, to avoid publishing clock==0");
        return;
    }

    // TODO make this only affect one robot if necessary
    if((this->base_watchdog_timeout.nanoseconds() > 0) &&
            ((this->sim_time - this->base_last_cmd) >= this->base_watchdog_timeout))
    {
        for (size_t r = 0; r < this->positionmodels.size(); r++)
            this->positionmodels[r]->SetSpeed(0.0, 0.0, 0.0);
    }

    //loop on the robot models
    for (size_t r = 0; r < this->robotmodels_.size(); ++r)
    {
        StageRobot const * robotmodel = this->robotmodels_[r];

        //loop on the laser devices for the current robot
        for (size_t s = 0; s < robotmodel->lasermodels.size(); ++s)
        {
            Stg::ModelRanger const* lasermodel = robotmodel->lasermodels[s];
            const std::vector<Stg::ModelRanger::Sensor>& sensors = lasermodel->GetSensors();

            if( sensors.size() > 1 )
                RCLCPP_WARN(this->n_->get_logger(), "ROS Stage currently supports rangers with 1 sensor only." );

            // for now we access only the zeroth sensor of the ranger - good
            // enough for most laser models that have a single beam origin
            const Stg::ModelRanger::Sensor& sensor = sensors[0];

            if( sensor.ranges.size() )
            {
                // Translate into ROS message format and publish
                sensor_msgs::msg::LaserScan msg;
                msg.angle_min = -sensor.fov/2.0;
                msg.angle_max = +sensor.fov/2.0;
                msg.angle_increment = sensor.fov/(double)(sensor.sample_count-1);
                msg.range_min = sensor.range.min;
                msg.range_max = sensor.range.max;
                msg.ranges.resize(sensor.ranges.size());
                msg.intensities.resize(sensor.intensities.size());

                for(unsigned int i = 0; i < sensor.ranges.size(); i++)
                {
                    msg.ranges[i] = sensor.ranges[i];
                    msg.intensities[i] = sensor.intensities[i];
                }

                if (robotmodel->lasermodels.size() > 1)
                    msg.header.frame_id = mapName("base_laser_link", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel));
                else
                    msg.header.frame_id = mapName("base_laser_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel));

                msg.header.stamp = sim_time;
                robotmodel->laser_pubs[s]->publish(msg);
            }

            // Also publish the base->base_laser_link Tx.  This could eventually move
            // into being retrieved from the param server as a static Tx.
            Stg::Pose lp = lasermodel->GetPose();
            tf2::Quaternion laserQ;
            laserQ.setRPY(0.0, 0.0, lp.a);
            tf2::Transform txLaser =  tf2::Transform(laserQ, tf2::Vector3(lp.x, lp.y, robotmodel->positionmodel->GetGeom().size.z + lp.z));

            if (robotmodel->lasermodels.size() > 1)
                tf.sendTransform(create_transform_stamped(txLaser, sim_time,
                                                      mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                                      mapName("base_laser_link", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel))));
            else
                tf.sendTransform(create_transform_stamped(txLaser, sim_time,
                                                      mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                                      mapName("base_laser_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel))));
        }

        //the position of the robot
        tf.sendTransform(create_transform_stamped(tf2::Transform::getIdentity(),
                                              sim_time,
                                              mapName("base_footprint", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                              mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel))));

        // Get latest odometry data
        // Translate into ROS message format and publish
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.pose.pose.position.x = robotmodel->positionmodel->est_pose.x;
        odom_msg.pose.pose.position.y = robotmodel->positionmodel->est_pose.y;
        odom_msg.pose.pose.orientation = createQuaternionMsgFromYaw(robotmodel->positionmodel->est_pose.a);
        Stg::Velocity v = robotmodel->positionmodel->GetVelocity();
        odom_msg.twist.twist.linear.x = v.x;
        odom_msg.twist.twist.linear.y = v.y;
        odom_msg.twist.twist.angular.z = v.a;

        //@todo Publish stall on a separate topic when one becomes available
        //this->odomMsgs[r].stall = this->positionmodels[r]->Stall();
        //
        odom_msg.header.frame_id = mapName("odom", r, static_cast<Stg::Model*>(robotmodel->positionmodel));
        odom_msg.header.stamp = sim_time;

        robotmodel->odom_pub->publish(odom_msg);

        // broadcast odometry transform
        tf2::Quaternion odomQ = tf2::Quaternion(
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w);
        tf2::Transform txOdom(odomQ, tf2::Vector3(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, 0.0));
        tf.sendTransform(create_transform_stamped(txOdom, sim_time,
                                              mapName("odom", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                              mapName("base_footprint", r, static_cast<Stg::Model*>(robotmodel->positionmodel))));

        // Also publish the ground truth pose and velocity
        Stg::Pose gpose = robotmodel->positionmodel->GetGlobalPose();
        tf2::Quaternion q_gpose;
        q_gpose.setRPY(0.0, 0.0, gpose.a);
        tf2::Transform gt(q_gpose, tf2::Vector3(gpose.x, gpose.y, 0.0));
        // Velocity is 0 by default and will be set only if there is previous pose and time delta>0
        Stg::Velocity gvel(0,0,0,0);
        if (this->base_last_globalpos.size()>r){
            Stg::Pose prevpose = this->base_last_globalpos.at(r);
            double dT = (this->sim_time-this->base_last_globalpos_time).seconds();
            if (dT>0)
                gvel = Stg::Velocity(
                            (gpose.x - prevpose.x)/dT,
                            (gpose.y - prevpose.y)/dT,
                            (gpose.z - prevpose.z)/dT,
                            Stg::normalize(gpose.a - prevpose.a)/dT
                            );
            this->base_last_globalpos.at(r) = gpose;
        }else //There are no previous readings, adding current pose...
            this->base_last_globalpos.push_back(gpose);

        nav_msgs::msg::Odometry ground_truth_msg;
        ground_truth_msg.pose.pose.position.x     = gt.getOrigin().x();
        ground_truth_msg.pose.pose.position.y     = gt.getOrigin().y();
        ground_truth_msg.pose.pose.position.z     = gt.getOrigin().z();
        ground_truth_msg.pose.pose.orientation.x  = gt.getRotation().x();
        ground_truth_msg.pose.pose.orientation.y  = gt.getRotation().y();
        ground_truth_msg.pose.pose.orientation.z  = gt.getRotation().z();
        ground_truth_msg.pose.pose.orientation.w  = gt.getRotation().w();
        ground_truth_msg.twist.twist.linear.x = gvel.x;
        ground_truth_msg.twist.twist.linear.y = gvel.y;
        ground_truth_msg.twist.twist.linear.z = gvel.z;
        ground_truth_msg.twist.twist.angular.z = gvel.a;

        ground_truth_msg.header.frame_id = mapName("odom", r, static_cast<Stg::Model*>(robotmodel->positionmodel));
        ground_truth_msg.header.stamp = sim_time;

        robotmodel->ground_truth_pub->publish(ground_truth_msg);

        //cameras
        for (size_t s = 0; s < robotmodel->cameramodels.size(); ++s)
        {
            Stg::ModelCamera* cameramodel = robotmodel->cameramodels[s];
            // Get latest image data
            // Translate into ROS message format and publish
            if (cameramodel->FrameColor())
            {
                sensor_msgs::msg::Image image_msg;

                image_msg.height = cameramodel->getHeight();
                image_msg.width = cameramodel->getWidth();
                image_msg.encoding = "rgba8";
                //this->imageMsgs[r].is_bigendian="";
                image_msg.step = image_msg.width*4;
                image_msg.data.resize(image_msg.width * image_msg.height * 4);

                memcpy(&(image_msg.data[0]), cameramodel->FrameColor(), image_msg.width * image_msg.height * 4);

                //invert the opengl weirdness
                int height = image_msg.height - 1;
                int linewidth = image_msg.width*4;

                char* temp = new char[linewidth];
                for (int y = 0; y < (height+1)/2; y++)
                {
                    memcpy(temp,&image_msg.data[y*linewidth],linewidth);
                    memcpy(&(image_msg.data[y*linewidth]),&(image_msg.data[(height-y)*linewidth]),linewidth);
                    memcpy(&(image_msg.data[(height-y)*linewidth]),temp,linewidth);
                }

                if (robotmodel->cameramodels.size() > 1)
                    image_msg.header.frame_id = mapName("camera", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel));
                else
                    image_msg.header.frame_id = mapName("camera", r,static_cast<Stg::Model*>(robotmodel->positionmodel));
                image_msg.header.stamp = sim_time;

                robotmodel->image_pubs[s]->publish(image_msg);
            }

            //Get latest depth data
            //Translate into ROS message format and publish
            //Skip if there are no subscribers
            if (cameramodel->FrameDepth())
            {
                sensor_msgs::msg::Image depth_msg;
                depth_msg.height = cameramodel->getHeight();
                depth_msg.width = cameramodel->getWidth();
                depth_msg.encoding = this->isDepthCanonical?sensor_msgs::image_encodings::TYPE_32FC1:sensor_msgs::image_encodings::TYPE_16UC1;
                //this->depthMsgs[r].is_bigendian="";
                int sz = this->isDepthCanonical?sizeof(float):sizeof(uint16_t);
                size_t len = depth_msg.width * depth_msg.height;
                depth_msg.step = depth_msg.width * sz;
                depth_msg.data.resize(len*sz);

                //processing data according to REP118
                if (this->isDepthCanonical){
                    double nearClip = cameramodel->getCamera().nearClip();
                    double farClip = cameramodel->getCamera().farClip();
                    memcpy(&(depth_msg.data[0]),cameramodel->FrameDepth(),len*sz);
                    float * data = (float*)&(depth_msg.data[0]);
                    for (size_t i=0;i<len;++i)
                        if(data[i]<=nearClip)
                            data[i] = -INFINITY;
                        else if(data[i]>=farClip)
                            data[i] = INFINITY;
                }
                else{
                    int nearClip = (int)(cameramodel->getCamera().nearClip() * 1000);
                    int farClip = (int)(cameramodel->getCamera().farClip() * 1000);
                    for (size_t i=0;i<len;++i){
                        int v = (int)(cameramodel->FrameDepth()[i]*1000);
                        if (v<=nearClip || v>=farClip) v = 0;
                        ((uint16_t*)&(depth_msg.data[0]))[i] = (uint16_t) ((v<=nearClip || v>=farClip) ? 0 : v );
                    }
                }

                //invert the opengl weirdness
                int height = depth_msg.height - 1;
                int linewidth = depth_msg.width*sz;

                char* temp = new char[linewidth];
                for (int y = 0; y < (height+1)/2; y++)
                {
                    memcpy(temp,&depth_msg.data[y*linewidth],linewidth);
                    memcpy(&(depth_msg.data[y*linewidth]),&(depth_msg.data[(height-y)*linewidth]),linewidth);
                    memcpy(&(depth_msg.data[(height-y)*linewidth]),temp,linewidth);
                }

                if (robotmodel->cameramodels.size() > 1)
                    depth_msg.header.frame_id = mapName("camera", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel));
                else
                    depth_msg.header.frame_id = mapName("camera", r, static_cast<Stg::Model*>(robotmodel->positionmodel));
                depth_msg.header.stamp = sim_time;
                robotmodel->depth_pubs[s]->publish(depth_msg);
            }

            //sending camera's tf and info only if image or depth topics are subscribed to
            if ((cameramodel->FrameColor())
                    || (cameramodel->FrameDepth()))
            {

                Stg::Pose lp = cameramodel->GetPose();
                tf2::Quaternion Q; Q.setRPY(
                            (cameramodel->getCamera().pitch()*M_PI/180.0)-M_PI,
                            0.0,
                            lp.a+(cameramodel->getCamera().yaw()*M_PI/180.0) - robotmodel->positionmodel->GetPose().a
                            );

                tf2::Transform tr =  tf2::Transform(Q, tf2::Vector3(lp.x, lp.y, robotmodel->positionmodel->GetGeom().size.z+lp.z));

                if (robotmodel->cameramodels.size() > 1)
                    tf.sendTransform(create_transform_stamped(tr, sim_time,
                                                          mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                                          mapName("camera", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel))));
                else
                    tf.sendTransform(create_transform_stamped(tr, sim_time,
                                                          mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                                          mapName("camera", r, static_cast<Stg::Model*>(robotmodel->positionmodel))));

                sensor_msgs::msg::CameraInfo camera_msg;
                if (robotmodel->cameramodels.size() > 1)
                    camera_msg.header.frame_id = mapName("camera", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel));
                else
                    camera_msg.header.frame_id = mapName("camera", r, static_cast<Stg::Model*>(robotmodel->positionmodel));
                camera_msg.header.stamp = sim_time;
                camera_msg.height = cameramodel->getHeight();
                camera_msg.width = cameramodel->getWidth();

                double fx,fy,cx,cy;
                cx = camera_msg.width / 2.0;
                cy = camera_msg.height / 2.0;
                double fovh = cameramodel->getCamera().horizFov()*M_PI/180.0;
                double fovv = cameramodel->getCamera().vertFov()*M_PI/180.0;
                //double fx_ = 1.43266615300557*this->cameramodels[r]->getWidth()/tan(fovh);
                //double fy_ = 1.43266615300557*this->cameramodels[r]->getHeight()/tan(fovv);
                fx = cameramodel->getWidth()/(2*tan(fovh/2));
                fy = cameramodel->getHeight()/(2*tan(fovv/2));

                //ROS_INFO("fx=%.4f,%.4f; fy=%.4f,%.4f", fx, fx_, fy, fy_);


                camera_msg.d.resize(4, 0.0);

                camera_msg.k[0] = fx;
                camera_msg.k[2] = cx;
                camera_msg.k[4] = fy;
                camera_msg.k[5] = cy;
                camera_msg.k[8] = 1.0;

                camera_msg.r[0] = 1.0;
                camera_msg.r[4] = 1.0;
                camera_msg.r[8] = 1.0;

                camera_msg.p[0] = fx;
                camera_msg.p[2] = cx;
                camera_msg.p[5] = fy;
                camera_msg.p[6] = cy;
                camera_msg.p[10] = 1.0;

                robotmodel->camera_pubs[s]->publish(camera_msg);

            }

        }
    }

    this->base_last_globalpos_time = this->sim_time;
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = sim_time;
    this->clock_pub_->publish(clock_msg);
}
