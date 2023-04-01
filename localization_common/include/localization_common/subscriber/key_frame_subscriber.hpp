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

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "localization_common/sensor_data/key_frame.hpp"
#include "rclcpp/rclcpp.hpp"

namespace localization_common
{
class KeyFrameSubscriber
{
public:
  KeyFrameSubscriber(rclcpp::Node::SharedPtr node, std::string topic_name, size_t buff_size);
  KeyFrameSubscriber() = default;
  void parse_data(std::deque<KeyFrame> & key_frame_buff);

private:
  void msg_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr key_frame_msg_ptr);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriber_;
  std::deque<KeyFrame> new_key_frame_;

  std::mutex buff_mutex_;
};
}  // namespace localization_common
