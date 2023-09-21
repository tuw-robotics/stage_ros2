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

#ifndef STAGE_ROS2__TRANSFORM_BROADCASTER_H_
#define STAGE_ROS2__TRANSFORM_BROADCASTER_H_

#include <tf2_ros/transform_broadcaster.h>

namespace stage_ros2
{

/** \brief This class provides an easy way to publish coordinate frame transform information.
 * It will handle all the messaging and stuffing of messages.  And the function prototypes lay out all the
 * necessary data needed for each message.  */

class TransformBroadcaster
{
public:
  /** \brief Node constructor */
  template<class NodeT, class AllocatorT = std::allocator<void>>
  TransformBroadcaster(
    NodeT && node,
    const char *topic = "/tf",
    const rclcpp::QoS & qos = tf2_ros::DynamicBroadcasterQoS(),
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options = [] () {
      rclcpp::PublisherOptionsWithAllocator<AllocatorT> options;
      options.qos_overriding_options = rclcpp::QosOverridingOptions{
        rclcpp::QosPolicyKind::Depth,
        rclcpp::QosPolicyKind::Durability,
        rclcpp::QosPolicyKind::History,
        rclcpp::QosPolicyKind::Reliability};
      return options;
    } ())
    : TransformBroadcaster(
      rclcpp::node_interfaces::get_node_parameters_interface(node),
      rclcpp::node_interfaces::get_node_topics_interface(node),
      qos,
      options,
      topic)
  {}

  /** \brief Node interfaces constructor */
  template<class AllocatorT = std::allocator<void>>
  TransformBroadcaster(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    const rclcpp::QoS & qos = tf2_ros::DynamicBroadcasterQoS(),
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options = [] () {
      rclcpp::PublisherOptionsWithAllocator<AllocatorT> options;
      options.qos_overriding_options = rclcpp::QosOverridingOptions{
        rclcpp::QosPolicyKind::Depth,
        rclcpp::QosPolicyKind::Durability,
        rclcpp::QosPolicyKind::History,
        rclcpp::QosPolicyKind::Reliability};
      return options;
    } (),
    const char *topic = "/tf")
  {
    publisher_ = rclcpp::create_publisher<tf2_msgs::msg::TFMessage>(
      node_parameters, node_topics, topic, qos, options);
  }

  /** \brief Send a TransformStamped message
   *
   * The transform ʰTₐ added is from `child_frame_id`, `a` to `header.frame_id`,
   * `h`. That is, position in `child_frame_id` ᵃp can be transformed to
   * position in `header.frame_id` ʰp such that ʰp = ʰTₐ ᵃp .
   *
   */
  TF2_ROS_PUBLIC
  void sendTransform(const geometry_msgs::msg::TransformStamped & msgtf)
  {
    std::vector<geometry_msgs::msg::TransformStamped> v1;
    v1.push_back(msgtf);
    sendTransform(v1);
  }

  /** \brief Send a vector of TransformStamped messages
   *
   * The transforms ʰTₐ added are from `child_frame_id`, `a` to `header.frame_id`,
   * `h`. That is, position in `child_frame_id` ᵃp can be transformed to
   * position in `header.frame_id` ʰp such that ʰp = ʰTₐ ᵃp .
   */
  TF2_ROS_PUBLIC
  void sendTransform(const std::vector<geometry_msgs::msg::TransformStamped> & msgtf)
  {
    tf2_msgs::msg::TFMessage message;
    for (std::vector<geometry_msgs::msg::TransformStamped>::const_iterator it = msgtf.begin();
      it != msgtf.end(); ++it)
    {
      message.transforms.push_back(*it);
    }
    publisher_->publish(message);
  }


private:
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;
};

}  // namespace stage_ros2

#endif  // STAGE_ROS2__TRANSFORM_BROADCASTER_H_