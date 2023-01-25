#include <stage_ros2/stage_node.hpp>

#include <chrono>
#include <memory>
#include <filesystem>

#define TOPIC_IMAGE "image"
#define TOPIC_DEPTH "depth"
#define TOPIC_CAMERA_INFO "camera_info"
#define FRAME_CAMERA "camera"

using std::placeholders::_1;

StageNode::Vehicle::Camera::Camera(
  unsigned int id, Stg::ModelCamera * m,
  std::shared_ptr<Vehicle> & v, StageNode * n)
: id_(id), model(m), vehicle(v), node(n) {}

unsigned int StageNode::Vehicle::Camera::id() const
{
  return id_;
}

void StageNode::Vehicle::Camera::init(bool add_id_to_topic)
{
  model->Subscribe();
  topic_name_image = vehicle->name_space_ + TOPIC_IMAGE;
  topic_name_camera_info = vehicle->name_space_ + TOPIC_CAMERA_INFO;
  topic_name_depth = vehicle->name_space_ + TOPIC_DEPTH;
  frame_id = vehicle->name_space_ + FRAME_CAMERA;

  if (add_id_to_topic) {
    topic_name_image += std::to_string(id());
    topic_name_camera_info += std::to_string(id());
    topic_name_depth += std::to_string(id());
    frame_id += std::to_string(id());
  }

  pub_image = node->create_publisher<sensor_msgs::msg::Image>(topic_name_image, 10);
  pub_camera = node->create_publisher<sensor_msgs::msg::CameraInfo>(topic_name_camera_info, 10);
  pub_depth = node->create_publisher<sensor_msgs::msg::Image>(topic_name_depth, 10);
}
bool StageNode::Vehicle::Camera::prepare_msg_image()
{
  if (msg_image) {
    return true;
  }
  if (!model->FrameColor()) {
    return false;
  }
  msg_image = std::make_shared<sensor_msgs::msg::Image>();

  msg_image->height = model->getHeight();
  msg_image->width = model->getWidth();
  msg_image->encoding = "rgba8";
  // node->imageMsgs[r].is_bigendian="";
  msg_image->step = msg_image->width * 4;
  msg_image->data.resize(msg_image->width * msg_image->height * 4);
  msg_image->header.frame_id = frame_id;

  return true;
}
bool StageNode::Vehicle::Camera::prepare_msg_depth()
{
  if (msg_depth) {
    return true;
  }
  if (!model->FrameDepth()) {
    return false;
  }
  msg_depth = std::make_shared<sensor_msgs::msg::Image>();
  msg_depth->height = model->getHeight();
  msg_depth->width = model->getWidth();
  msg_depth->encoding =
    node->isDepthCanonical_ ? sensor_msgs::image_encodings::TYPE_32FC1 : sensor_msgs::
    image_encodings::TYPE_16UC1;
  // node->depthMsgs[r].is_bigendian="";
  int sz = node->isDepthCanonical_ ? sizeof(float) : sizeof(uint16_t);
  size_t len = msg_depth->width * msg_depth->height;
  msg_depth->step = msg_depth->width * sz;
  msg_depth->data.resize(len * sz);
  return true;
}
bool StageNode::Vehicle::Camera::prepare_msg_camera()
{

  if (msg_camera) {
    return true;
  }
  if (!(model->FrameColor() || model->FrameDepth())) {
    return false;
  }
  msg_camera = std::make_shared<sensor_msgs::msg::CameraInfo>();
  msg_camera->header.frame_id = frame_id;
  msg_camera->header.stamp = node->sim_time_;
  msg_camera->height = model->getHeight();
  msg_camera->width = model->getWidth();

  double fx, fy, cx, cy;
  cx = msg_camera->width / 2.0;
  cy = msg_camera->height / 2.0;
  double fovh = model->getCamera().horizFov() * M_PI / 180.0;
  double fovv = model->getCamera().vertFov() * M_PI / 180.0;
  // double fx_ = 1.43266615300557*node->models[r]->getWidth()/tan(fovh);
  // double fy_ = 1.43266615300557*node->models[r]->getHeight()/tan(fovv);
  fx = model->getWidth() / (2 * tan(fovh / 2));
  fy = model->getHeight() / (2 * tan(fovv / 2));

  // ROS_INFO("fx=%.4f,%.4f; fy=%.4f,%.4f", fx, fx_, fy, fy_);

  msg_camera->d.resize(4, 0.0);

  msg_camera->k[0] = fx;
  msg_camera->k[2] = cx;
  msg_camera->k[4] = fy;
  msg_camera->k[5] = cy;
  msg_camera->k[8] = 1.0;

  msg_camera->r[0] = 1.0;
  msg_camera->r[4] = 1.0;
  msg_camera->r[8] = 1.0;

  msg_camera->p[0] = fx;
  msg_camera->p[2] = cx;
  msg_camera->p[5] = fy;
  msg_camera->p[6] = cy;
  msg_camera->p[10] = 1.0;
  return true;
}

bool StageNode::Vehicle::Camera::prepare_msg()
{
  if (!prepare_msg_image()) {return false;}
  if (!prepare_msg_depth()) {return false;}
  return true;
}

void StageNode::Vehicle::Camera::publish_msg()
{
  // Translate into ROS message format and publish
  if (prepare_msg_image()) {

    memcpy(&(msg_image->data[0]), model->FrameColor(), msg_image->width * msg_image->height * 4);

    // invert the opengl weirdness
    char * temp = new char[msg_image->step];
    for (unsigned int y = 0; y < (msg_image->height + 1) / 2; y++) {
      memcpy(temp, &msg_image->data[y * msg_image->step], msg_image->step);
      memcpy(
        &(msg_image->data[y * msg_image->step]),
        &(msg_image->data[(msg_image->height - y - 1) * msg_image->step]), msg_image->step);
      memcpy(
        &(msg_image->data[(msg_image->height - y - 1) * msg_image->step]), temp,
        msg_image->step);
    }
    msg_image->header.stamp = node->sim_time_;
    pub_image->publish(*msg_image);
  }

  // Translate into ROS message format and publish
  if (prepare_msg_depth()) {
    // processing data according to REP118
    if (node->isDepthCanonical_) {
      float nearClip = model->getCamera().nearClip();
      float farClip = model->getCamera().farClip();
      memcpy(&(msg_depth->data[0]), model->FrameDepth(), msg_depth->data.size());
      float * data = (float *)&(msg_depth->data[0]);
      size_t len = msg_depth->width * msg_depth->height;
      for (size_t i = 0; i < len; ++i) {
        if (data[i] <= nearClip) {
          data[i] = -INFINITY;
        } else if (data[i] >= farClip) {
          data[i] = INFINITY;
        }
      }
    } else {
      int nearClip = (int)(model->getCamera().nearClip() * 1000);
      int farClip = (int)(model->getCamera().farClip() * 1000);
      size_t len = msg_depth->width * msg_depth->height;
      for (size_t i = 0; i < len; ++i) {
        int v = (int)(model->FrameDepth()[i] * 1000);
        if (v <= nearClip || v >= farClip) {
          v = 0;
        }
        ((uint16_t *)&(msg_depth->data[0]))[i] =
          (uint16_t)((v <= nearClip || v >= farClip) ? 0 : v);
      }
    }

    // invert the opengl weirdness
    char * temp = new char[msg_depth->step];
    for (unsigned int y = 0; y < msg_depth->height / 2; y++) {
      memcpy(temp, &msg_depth->data[y * msg_depth->step], msg_depth->step);
      memcpy(
        &(msg_depth->data[y * msg_depth->step]),
        &(msg_depth->data[(msg_depth->height - 1 - y) * msg_depth->step]), msg_depth->step);
      memcpy(
        &(msg_depth->data[(msg_depth->height - 1 - y) * msg_depth->step]), temp,
        msg_depth->step);
    }

    msg_depth->header.frame_id = frame_id;
    msg_depth->header.stamp = node->sim_time_;
    pub_depth->publish(*msg_depth);
  }

  // Translate into ROS message format and publish
  if (prepare_msg_camera()) {
    msg_camera->header.stamp = node->sim_time_;
    pub_camera->publish(*msg_camera);
  }
}
bool StageNode::Vehicle::Camera::prepare_tf()
{
  if (transform) {return true;}

  if (!(model->FrameColor() || model->FrameDepth())) {return false;}
  transform = std::make_shared<geometry_msgs::msg::TransformStamped>();

  Stg::Pose pose = model->GetPose();
  tf2::Quaternion quternion;
  quternion.setRPY(
    (model->getCamera().pitch() * M_PI / 180.0) - M_PI,
    0.0,
    pose.a + (model->getCamera().yaw() * M_PI / 180.0) - vehicle->positionmodel->GetPose().a);
  tf2::Transform tr =
    tf2::Transform(
    quternion,
    tf2::Vector3(pose.x, pose.y, vehicle->positionmodel->GetGeom().size.z + pose.z));
  *transform =
    create_transform_stamped(tr, node->sim_time_, vehicle->frame_id_base_link_, frame_id);
  if (node->use_static_transformations_) {
    vehicle->tf_static_broadcaster_->sendTransform(*transform);
  }
  return true;
}
void StageNode::Vehicle::Camera::publish_tf()
{
  if (prepare_tf()) {

    if (node->use_static_transformations_) {return;}

    // use tf publsiher only if use_static_transformations_ is false
    transform->header.stamp = node->sim_time_;
    vehicle->tf_broadcaster_->sendTransform(*transform);
  }
}
