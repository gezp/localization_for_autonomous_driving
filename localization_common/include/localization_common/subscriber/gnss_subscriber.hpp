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
#include <string>

#include "localization_common/sensor_data/gnss_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace localization_common
{
class GNSSSubscriber
{
public:
  GNSSSubscriber(rclcpp::Node::SharedPtr node, std::string topic_name, size_t buff_size);
  GNSSSubscriber() = default;
  void parse_data(std::deque<GNSSData> & deque_gnss_data);

private:
  void msg_callback(const sensor_msgs::msg::NavSatFix::SharedPtr nav_sat_fix_ptr);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber_;

  std::deque<GNSSData> new_gnss_data_;
  std::mutex buff_mutex_;
};
}  // namespace localization_common
