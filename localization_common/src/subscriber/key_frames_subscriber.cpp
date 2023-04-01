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

#include "localization_common/subscriber/key_frames_subscriber.hpp"

namespace localization_common
{
KeyFramesSubscriber::KeyFramesSubscriber(
  rclcpp::Node::SharedPtr node, std::string topic_name, size_t buff_size)
: node_(node)
{
  subscriber_ = node_->create_subscription<nav_msgs::msg::Path>(
    topic_name, buff_size,
    std::bind(&KeyFramesSubscriber::msg_callback, this, std::placeholders::_1));
}

void KeyFramesSubscriber::msg_callback(const nav_msgs::msg::Path::SharedPtr key_frames_msg_ptr)
{
  buff_mutex_.lock();
  new_key_frames_.clear();

  for (size_t i = 0; i < key_frames_msg_ptr->poses.size(); i++) {
    KeyFrame key_frame;
    key_frame.time = rclcpp::Time(key_frames_msg_ptr->poses.at(i).header.stamp).seconds();
    key_frame.index = (unsigned int)i;

    key_frame.pose(0, 3) = key_frames_msg_ptr->poses.at(i).pose.position.x;
    key_frame.pose(1, 3) = key_frames_msg_ptr->poses.at(i).pose.position.y;
    key_frame.pose(2, 3) = key_frames_msg_ptr->poses.at(i).pose.position.z;

    Eigen::Quaternionf q;
    q.x() = key_frames_msg_ptr->poses.at(i).pose.orientation.x;
    q.y() = key_frames_msg_ptr->poses.at(i).pose.orientation.y;
    q.z() = key_frames_msg_ptr->poses.at(i).pose.orientation.z;
    q.w() = key_frames_msg_ptr->poses.at(i).pose.orientation.w;
    key_frame.pose.block<3, 3>(0, 0) = q.matrix();

    new_key_frames_.push_back(key_frame);
  }
  buff_mutex_.unlock();
}

void KeyFramesSubscriber::parse_data(std::deque<KeyFrame> & key_frames_buff)
{
  buff_mutex_.lock();
  if (new_key_frames_.size() > 0) {
    key_frames_buff = new_key_frames_;
    new_key_frames_.clear();
  }
  buff_mutex_.unlock();
}
}  // namespace localization_common
