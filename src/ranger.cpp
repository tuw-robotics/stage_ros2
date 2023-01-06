#include <stage_ros2/stage_node.hpp>

#include <chrono>
#include <memory>
#include <filesystem>

#define TOPIC_LASER "base_scan"
#define FRAME_LASER "laser"

using std::placeholders::_1;

StageNode::Vehicle::Ranger::Ranger(Stg::ModelRanger *m, std::shared_ptr<Vehicle> &v, StageNode *n)
    : model(m), vehicle(v), node(n){};

void StageNode::Vehicle::Ranger::init(bool add_id_to_topic)
{
    model->Subscribe();
    topic_name = vehicle->name_space_ + TOPIC_LASER;
    frame_id = vehicle->name_space_ + FRAME_LASER;
    if (add_id_to_topic)
    {
        topic_name += std::to_string(id);
        frame_id += std::to_string(id);
    }

    pub = node->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 10);
}

void StageNode::Vehicle::Ranger::publish_msg()
{
    const std::vector<Stg::ModelRanger::Sensor> &sensors = model->GetSensors();

    if (sensors.size() > 1)
        RCLCPP_WARN(node->get_logger(), "ROS Stage currently supports rangers with 1 sensor only.");

    // for now we access only the zeroth sensor of the ranger - good
    // enough for most laser models that have a single beam origin
    const Stg::ModelRanger::Sensor &sensor = sensors[0];

    if (sensor.ranges.size())
    {
        msg.angle_min = -sensor.fov / 2.0;
        msg.angle_max = +sensor.fov / 2.0;
        msg.angle_increment = sensor.fov / (double)(sensor.sample_count - 1);
        msg.range_min = sensor.range.min;
        msg.range_max = sensor.range.max;
        msg.ranges.resize(sensor.ranges.size());
        msg.intensities.resize(sensor.intensities.size());
        msg.header.frame_id = frame_id;
    }

    msg.header.stamp = node->sim_time_;
    for (unsigned int i = 0; i < sensor.ranges.size(); i++)
    {
        msg.ranges[i] = sensor.ranges[i];
        msg.intensities[i] = sensor.intensities[i];
    }
    pub->publish(msg);
}

void StageNode::Vehicle::Ranger::publish_tf()
{
    // Also publish the base->base_laser_link Tf.  This could eventually move
    // into being retrieved from the param server as a static Tx.
    Stg::Pose pose = model->GetPose();
    tf2::Quaternion quternion;
    quternion.setRPY(0.0, 0.0, pose.a);
    tf2::Transform txLaser = tf2::Transform(quternion, tf2::Vector3(pose.x, pose.y, vehicle->positionmodel->GetGeom().size.z + pose.z));
    transform = create_transform_stamped(txLaser, node->sim_time_, vehicle->frame_id_base_, frame_id);
    node->tf_->sendTransform(transform);
}