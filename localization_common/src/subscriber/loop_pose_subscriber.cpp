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

#include "localization_common/subscriber/loop_pose_subscriber.hpp"

namespace localization_common
{
LoopPoseSubscriber::LoopPoseSubscriber(
  rclcpp::Node::SharedPtr node, std::string topic_name, size_t buff_size)
: node_(node)
{
  subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    topic_name, buff_size,
    std::bind(&LoopPoseSubscriber::msg_callback, this, std::placeholders::_1));
}

void LoopPoseSubscriber::msg_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr loop_pose_msg_ptr)
{
  buff_mutex_.lock();
  LoopPose loop_pose;
  loop_pose.time = rclcpp::Time(loop_pose_msg_ptr->header.stamp).seconds();
  loop_pose.index0 = (unsigned int)loop_pose_msg_ptr->pose.covariance[0];
  loop_pose.index1 = (unsigned int)loop_pose_msg_ptr->pose.covariance[1];

  loop_pose.pose(0, 3) = loop_pose_msg_ptr->pose.pose.position.x;
  loop_pose.pose(1, 3) = loop_pose_msg_ptr->pose.pose.position.y;
  loop_pose.pose(2, 3) = loop_pose_msg_ptr->pose.pose.position.z;

  Eigen::Quaternionf q;
  q.x() = loop_pose_msg_ptr->pose.pose.orientation.x;
  q.y() = loop_pose_msg_ptr->pose.pose.orientation.y;
  q.z() = loop_pose_msg_ptr->pose.pose.orientation.z;
  q.w() = loop_pose_msg_ptr->pose.pose.orientation.w;
  loop_pose.pose.block<3, 3>(0, 0) = q.matrix();

  new_loop_pose_.push_back(loop_pose);
  buff_mutex_.unlock();
}

void LoopPoseSubscriber::parse_data(std::deque<LoopPose> & loop_pose_buff)
{
  buff_mutex_.lock();
  if (new_loop_pose_.size() > 0) {
    loop_pose_buff.insert(loop_pose_buff.end(), new_loop_pose_.begin(), new_loop_pose_.end());
    new_loop_pose_.clear();
  }
  buff_mutex_.unlock();
}
}  // namespace localization_common
