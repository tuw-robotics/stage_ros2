#ifndef STAGE_ROS2_PKG__CUSTOM_TF_BROADCASTER_HPP_
#define STAGE_ROS2_PKG__CUSTOM_TF_BROADCASTER_HPP_

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace stage_ros2
{

  class StaticTransformBroadcaster
  {
  public:
    /** \brief Node interface constructor */
    template <class NodeT, class AllocatorT = std::allocator<void>>
    StaticTransformBroadcaster(
        NodeT &&node,
        const rclcpp::QoS &qos = tf2_ros::StaticBroadcasterQoS(),
        const rclcpp::PublisherOptionsWithAllocator<AllocatorT> &options = []()
        {
      rclcpp::PublisherOptionsWithAllocator<AllocatorT> options;
      options.qos_overriding_options = rclcpp::QosOverridingOptions{
        rclcpp::QosPolicyKind::Depth,
        rclcpp::QosPolicyKind::History,
        rclcpp::QosPolicyKind::Reliability};
      /*
        This flag disables intra-process communication while publishing to
        /tf_static topic, when the StaticTransformBroadcaster is constructed
        using an existing node handle which happens to be a component
        (in rclcpp terminology).
        Required until rclcpp intra-process communication supports
        transient_local QoS durability.
      */
      options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
      return options; }())
    {
      publisher_ = rclcpp::create_publisher<tf2_msgs::msg::TFMessage>(
          node, "/tf_static", qos, options);
    }

    /** \brief Send a TransformStamped message
     * The stamped data structure includes frame_id, and time, and parent_id already.  */
    TF2_ROS_PUBLIC
    void sendTransform(const geometry_msgs::msg::TransformStamped &msgtf)
    {
      std::vector<geometry_msgs::msg::TransformStamped> v1;
      v1.push_back(msgtf);
      sendTransform(v1);
    }

    /** \brief Send a vector of TransformStamped messages
     * The stamped data structure includes frame_id, and time, and parent_id already.  */
    TF2_ROS_PUBLIC
    void sendTransform(const std::vector<geometry_msgs::msg::TransformStamped> &msgtf)
    {
      for (auto it_in = msgtf.begin(); it_in != msgtf.end(); ++it_in)
      {
        bool match_found = false;
        for (auto it_msg = net_message_.transforms.begin(); it_msg != net_message_.transforms.end();
             ++it_msg)
        {
          if (it_in->child_frame_id == it_msg->child_frame_id)
          {
            *it_msg = *it_in;
            match_found = true;
            break;
          }
        }
        if (!match_found)
        {
          net_message_.transforms.push_back(*it_in);
        }
      }

      publisher_->publish(net_message_);
    }

  private:
    /// Internal reference to ros::Node
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;
    tf2_msgs::msg::TFMessage net_message_;
  };

} // namespace stage_ros2
#endif // STAGE_ROS2_PKG__CUSTOM_TF_BROADCASTER_HPP_