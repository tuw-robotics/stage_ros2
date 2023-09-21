/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


/** \author Tully Foote */
/** \author Markus Bader (2023) */

#ifndef STAGE_ROS2_PKG__CUSTOM_TF_BROADCASTER_HPP_
#define STAGE_ROS2_PKG__CUSTOM_TF_BROADCASTER_HPP_

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
        const char *topic = "/tf_static",
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
          node, topic, qos, options);
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