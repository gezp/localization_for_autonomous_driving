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

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
//
#include "localization_common/sensor_data/cloud_data.hpp"

namespace localization_common
{
class CloudPublisher
{
public:
  CloudPublisher(
    rclcpp::Node::SharedPtr node, std::string topic_name, std::string frame_id, size_t buff_size);
  void publish(PointXYZCloudPtr cloud_input, rclcpp::Time time);
  void publish(PointXYZCloudPtr cloud_input, double time);
  void publish(PointXYZCloudPtr cloud_input);

  bool has_subscribers() {return publisher_->get_subscription_count() > 0;}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::string frame_id_;
};
}  // namespace localization_common
