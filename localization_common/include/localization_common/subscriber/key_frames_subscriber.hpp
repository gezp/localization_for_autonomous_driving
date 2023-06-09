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

#pragma once

#include <deque>
#include <mutex>
#include <thread>
#include <string>

#include "localization_common/sensor_data/key_frame.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace localization_common
{
class KeyFramesSubscriber
{
public:
  KeyFramesSubscriber(rclcpp::Node::SharedPtr node, std::string topic_name, size_t buff_size);
  KeyFramesSubscriber() = default;
  void parse_data(std::deque<KeyFrame> & deque_key_frames);

private:
  void msg_callback(const nav_msgs::msg::Path::SharedPtr key_frames_msg_ptr);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscriber_;
  std::deque<KeyFrame> new_key_frames_;

  std::mutex buff_mutex_;
};
}  // namespace localization_common
