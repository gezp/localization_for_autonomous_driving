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
  rclcpp::Node::SharedPtr node, std::string topic_name, size_t buff_size)
: node_(node)
{
  subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    topic_name, buff_size,
    std::bind(&OdometrySubscriber::msg_callback, this, std::placeholders::_1));
}

void OdometrySubscriber::msg_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg_ptr)
{
  buff_mutex_.lock();
  PoseData pose_data;
  pose_data.time = rclcpp::Time(odom_msg_ptr->header.stamp).seconds();

  // set the position:
  pose_data.pose(0, 3) = odom_msg_ptr->pose.pose.position.x;
  pose_data.pose(1, 3) = odom_msg_ptr->pose.pose.position.y;
  pose_data.pose(2, 3) = odom_msg_ptr->pose.pose.position.z;

  // set the orientation:
  Eigen::Quaternionf q;
  q.x() = odom_msg_ptr->pose.pose.orientation.x;
  q.y() = odom_msg_ptr->pose.pose.orientation.y;
  q.z() = odom_msg_ptr->pose.pose.orientation.z;
  q.w() = odom_msg_ptr->pose.pose.orientation.w;
  pose_data.pose.block<3, 3>(0, 0) = q.matrix();

  // set the linear velocity:
  pose_data.vel.v.x() = odom_msg_ptr->twist.twist.linear.x;
  pose_data.vel.v.y() = odom_msg_ptr->twist.twist.linear.y;
  pose_data.vel.v.z() = odom_msg_ptr->twist.twist.linear.z;

  // set the angular velocity:
  pose_data.vel.w.x() = odom_msg_ptr->twist.twist.angular.x;
  pose_data.vel.w.y() = odom_msg_ptr->twist.twist.angular.y;
  pose_data.vel.w.z() = odom_msg_ptr->twist.twist.angular.z;

  new_pose_data_.push_back(pose_data);

  buff_mutex_.unlock();
}

void OdometrySubscriber::parse_data(std::deque<PoseData> & pose_data_buff)
{
  buff_mutex_.lock();
  if (new_pose_data_.size() > 0) {
    pose_data_buff.insert(pose_data_buff.end(), new_pose_data_.begin(), new_pose_data_.end());
    new_pose_data_.clear();
  }
  buff_mutex_.unlock();
}

}  // namespace localization_common
