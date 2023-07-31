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
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
//
#include "localization_common/sensor_data/key_frame.hpp"
#include "localization_common/sensor_data/lidar_frame.hpp"

namespace localization_common
{
class PathPublisher
{
public:
  PathPublisher(
    rclcpp::Node::SharedPtr node, std::string topic_name, std::string frame_id, int buff_size);

  void publish(const std::deque<KeyFrame> & key_frames);
  void publish(const std::vector<LidarFrame> & key_frames);
  bool has_subscribers() {return publisher_->get_subscription_count() > 0;}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  std::string frame_id_ = "";
};
}  // namespace localization_common
