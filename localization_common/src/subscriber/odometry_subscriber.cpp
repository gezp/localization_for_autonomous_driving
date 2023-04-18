// Copyright 2023 Gezp (https://github.com/gezp).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "localization_common/subscriber/odometry_subscriber.hpp"

namespace localization_common
{

OdometrySubscriber::OdometrySubscriber(
  rclcpp::Node::SharedPtr node, std::string topic_name, size_t buffer_size)
: node_(node)
{
  subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    topic_name, buffer_size,
    std::bind(&OdometrySubscriber::msg_callback, this, std::placeholders::_1));
}

void OdometrySubscriber::msg_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  OdomData odom_data;
  odom_data.time = rclcpp::Time(odom_msg->header.stamp).seconds();

  // set the position:
  odom_data.pose(0, 3) = odom_msg->pose.pose.position.x;
  odom_data.pose(1, 3) = odom_msg->pose.pose.position.y;
  odom_data.pose(2, 3) = odom_msg->pose.pose.position.z;

  // set the orientation:
  Eigen::Quaternionf q;
  q.x() = odom_msg->pose.pose.orientation.x;
  q.y() = odom_msg->pose.pose.orientation.y;
  q.z() = odom_msg->pose.pose.orientation.z;
  q.w() = odom_msg->pose.pose.orientation.w;
  odom_data.pose.block<3, 3>(0, 0) = q.matrix();

  // set the linear velocity:
  odom_data.linear_velocity.x() = odom_msg->twist.twist.linear.x;
  odom_data.linear_velocity.y() = odom_msg->twist.twist.linear.y;
  odom_data.linear_velocity.z() = odom_msg->twist.twist.linear.z;

  // set the angular velocity:
  odom_data.angular_velocity.x() = odom_msg->twist.twist.angular.x;
  odom_data.angular_velocity.y() = odom_msg->twist.twist.angular.y;
  odom_data.angular_velocity.z() = odom_msg->twist.twist.angular.z;

  buffer_mutex_.lock();
  odom_data_buffer_.push_back(odom_data);
  buffer_mutex_.unlock();
}

void OdometrySubscriber::parse_data(std::deque<OdomData> & odom_data_buffer)
{
  buffer_mutex_.lock();
  if (odom_data_buffer_.size() > 0) {
    odom_data_buffer.insert(
      odom_data_buffer.end(), odom_data_buffer_.begin(), odom_data_buffer_.end());
    odom_data_buffer_.clear();
  }
  buffer_mutex_.unlock();
}

void OdometrySubscriber::parse_data(std::deque<PoseData> & pose_data_buffer)
{
  buffer_mutex_.lock();
  if (odom_data_buffer_.size() > 0) {
    for (auto & odom_data : odom_data_buffer_) {
      PoseData pose_data;
      pose_data.pose = odom_data.pose;
      pose_data.vel.v = odom_data.linear_velocity;
      pose_data.vel.w = odom_data.angular_velocity;
      pose_data_buffer.push_back(pose_data);
    }
    odom_data_buffer_.clear();
  }
  buffer_mutex_.unlock();
}

}  // namespace localization_common
