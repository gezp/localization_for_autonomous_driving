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

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
//
#include "localization_common/sensor_data/imu_data.hpp"


namespace localization_common
{
class ImuPublisher
{
public:
  ImuPublisher(
    rclcpp::Node::SharedPtr node, std::string topic_name, std::string frame_id, size_t buff_size);

  void publish(const ImuData & imu_data, rclcpp::Time time);
  void publish(const ImuData & imu_data, double time);
  void publish(const ImuData & imu_data);
  bool has_subscribers(void);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  std::string frame_id_;

  sensor_msgs::msg::Imu imu_;
};
}  // namespace localization_common
