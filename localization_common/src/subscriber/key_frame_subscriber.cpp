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

#include "localization_common/subscriber/key_frame_subscriber.hpp"

namespace localization_common
{
KeyFrameSubscriber::KeyFrameSubscriber(
  rclcpp::Node::SharedPtr node, std::string topic_name, size_t buff_size)
: node_(node)
{
  subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    topic_name, buff_size,
    std::bind(&KeyFrameSubscriber::msg_callback, this, std::placeholders::_1));
}

void KeyFrameSubscriber::msg_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr key_frame_msg_ptr)
{
  buff_mutex_.lock();
  KeyFrame key_frame;
  key_frame.time = rclcpp::Time(key_frame_msg_ptr->header.stamp).seconds();
  key_frame.index = (unsigned int)key_frame_msg_ptr->pose.covariance[0];

  key_frame.pose(0, 3) = key_frame_msg_ptr->pose.pose.position.x;
  key_frame.pose(1, 3) = key_frame_msg_ptr->pose.pose.position.y;
  key_frame.pose(2, 3) = key_frame_msg_ptr->pose.pose.position.z;

  Eigen::Quaternionf q;
  q.x() = key_frame_msg_ptr->pose.pose.orientation.x;
  q.y() = key_frame_msg_ptr->pose.pose.orientation.y;
  q.z() = key_frame_msg_ptr->pose.pose.orientation.z;
  q.w() = key_frame_msg_ptr->pose.pose.orientation.w;
  key_frame.pose.block<3, 3>(0, 0) = q.matrix();

  new_key_frame_.push_back(key_frame);
  buff_mutex_.unlock();
}

void KeyFrameSubscriber::parse_data(std::deque<KeyFrame> & key_frame_buff)
{
  buff_mutex_.lock();
  if (new_key_frame_.size() > 0) {
    key_frame_buff.insert(key_frame_buff.end(), new_key_frame_.begin(), new_key_frame_.end());
    new_key_frame_.clear();
  }
  buff_mutex_.unlock();
}
}  // namespace localization_common
