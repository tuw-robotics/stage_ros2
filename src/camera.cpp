#include <stage_ros2/stage_node.hpp>

#include <chrono>
#include <memory>
#include <filesystem>

#define TOPIC_IMAGE "image"
#define TOPIC_DEPTH "depth"
#define TOPIC_CAMERA_INFO "camera_info"
#define FRAME_CAMERA "camera"

using std::placeholders::_1;

StageNode::Vehicle::Camera::Camera(unsigned int id, Stg::ModelCamera *m, std::shared_ptr<Vehicle> &v, StageNode *n)
    : id_(id), model(m), vehicle(v), node(n){};

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

    if (add_id_to_topic)
    {
        topic_name_image += std::to_string(id());
        topic_name_camera_info += std::to_string(id());
        topic_name_depth += std::to_string(id());
        frame_id += std::to_string(id());
    }

    pub_image = node->create_publisher<sensor_msgs::msg::Image>(topic_name_image, 10);
    pub_camera = node->create_publisher<sensor_msgs::msg::CameraInfo>(topic_name_camera_info, 10);
    pub_depth = node->create_publisher<sensor_msgs::msg::Image>(topic_name_depth, 10);
}
void StageNode::Vehicle::Camera::publish_msg()
{
    if (model->FrameColor())
    {
        sensor_msgs::msg::Image image_msg;

        image_msg.height = model->getHeight();
        image_msg.width = model->getWidth();
        image_msg.encoding = "rgba8";
        // node->imageMsgs[r].is_bigendian="";
        image_msg.step = image_msg.width * 4;
        image_msg.data.resize(image_msg.width * image_msg.height * 4);

        memcpy(&(image_msg.data[0]), model->FrameColor(), image_msg.width * image_msg.height * 4);

        // invert the opengl weirdness
        int height = image_msg.height - 1;
        int linewidth = image_msg.width * 4;

        char *temp = new char[linewidth];
        for (int y = 0; y < (height + 1) / 2; y++)
        {
            memcpy(temp, &image_msg.data[y * linewidth], linewidth);
            memcpy(&(image_msg.data[y * linewidth]), &(image_msg.data[(height - y) * linewidth]), linewidth);
            memcpy(&(image_msg.data[(height - y) * linewidth]), temp, linewidth);
        }

        image_msg.header.frame_id = frame_id;
        image_msg.header.stamp = node->sim_time_;

        pub_image->publish(image_msg);
    }

    // Get latest depth data
    // Translate into ROS message format and publish
    // Skip if there are no subscribers
    if (model->FrameDepth())
    {
        sensor_msgs::msg::Image depth_msg;
        depth_msg.height = model->getHeight();
        depth_msg.width = model->getWidth();
        depth_msg.encoding = node->isDepthCanonical_ ? sensor_msgs::image_encodings::TYPE_32FC1 : sensor_msgs::image_encodings::TYPE_16UC1;
        // node->depthMsgs[r].is_bigendian="";
        int sz = node->isDepthCanonical_ ? sizeof(float) : sizeof(uint16_t);
        size_t len = depth_msg.width * depth_msg.height;
        depth_msg.step = depth_msg.width * sz;
        depth_msg.data.resize(len * sz);

        // processing data according to REP118
        if (node->isDepthCanonical_)
        {
            double nearClip = model->getCamera().nearClip();
            double farClip = model->getCamera().farClip();
            memcpy(&(depth_msg.data[0]), model->FrameDepth(), len * sz);
            float *data = (float *)&(depth_msg.data[0]);
            for (size_t i = 0; i < len; ++i)
                if (data[i] <= nearClip)
                    data[i] = -INFINITY;
                else if (data[i] >= farClip)
                    data[i] = INFINITY;
        }
        else
        {
            int nearClip = (int)(model->getCamera().nearClip() * 1000);
            int farClip = (int)(model->getCamera().farClip() * 1000);
            for (size_t i = 0; i < len; ++i)
            {
                int v = (int)(model->FrameDepth()[i] * 1000);
                if (v <= nearClip || v >= farClip)
                    v = 0;
                ((uint16_t *)&(depth_msg.data[0]))[i] = (uint16_t)((v <= nearClip || v >= farClip) ? 0 : v);
            }
        }

        // invert the opengl weirdness
        int height = depth_msg.height - 1;
        int linewidth = depth_msg.width * sz;

        char *temp = new char[linewidth];
        for (int y = 0; y < (height + 1) / 2; y++)
        {
            memcpy(temp, &depth_msg.data[y * linewidth], linewidth);
            memcpy(&(depth_msg.data[y * linewidth]), &(depth_msg.data[(height - y) * linewidth]), linewidth);
            memcpy(&(depth_msg.data[(height - y) * linewidth]), temp, linewidth);
        }

        depth_msg.header.frame_id = frame_id;
        depth_msg.header.stamp = node->sim_time_;
        pub_depth->publish(depth_msg);
    }

    // sending camera's tf and info only if image or depth topics are subscribed to
    if ((model->FrameColor()) || (model->FrameDepth()))
    {

        Stg::Pose lp = model->GetPose();
        tf2::Quaternion Q;
        Q.setRPY(
            (model->getCamera().pitch() * M_PI / 180.0) - M_PI,
            0.0,
            lp.a + (model->getCamera().yaw() * M_PI / 180.0) - vehicle->positionmodel->GetPose().a);

        tf2::Transform tr = tf2::Transform(Q, tf2::Vector3(lp.x, lp.y, vehicle->positionmodel->GetGeom().size.z + lp.z));
        node->tf_->sendTransform(create_transform_stamped(tr, node->sim_time_, vehicle->frame_id_base_, frame_id));

        sensor_msgs::msg::CameraInfo camera_msg;
        camera_msg.header.frame_id = frame_id;
        camera_msg.header.stamp = node->sim_time_;
        camera_msg.height = model->getHeight();
        camera_msg.width = model->getWidth();

        double fx, fy, cx, cy;
        cx = camera_msg.width / 2.0;
        cy = camera_msg.height / 2.0;
        double fovh = model->getCamera().horizFov() * M_PI / 180.0;
        double fovv = model->getCamera().vertFov() * M_PI / 180.0;
        // double fx_ = 1.43266615300557*node->models[r]->getWidth()/tan(fovh);
        // double fy_ = 1.43266615300557*node->models[r]->getHeight()/tan(fovv);
        fx = model->getWidth() / (2 * tan(fovh / 2));
        fy = model->getHeight() / (2 * tan(fovv / 2));

        // ROS_INFO("fx=%.4f,%.4f; fy=%.4f,%.4f", fx, fx_, fy, fy_);

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

        pub_camera->publish(camera_msg);
    }
}
void StageNode::Vehicle::Camera::publish_tf()
{
}