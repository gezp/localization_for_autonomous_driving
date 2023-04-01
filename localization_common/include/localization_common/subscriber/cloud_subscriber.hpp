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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <deque>
#include <mutex>
#include <string>

#include "localization_common/sensor_data/cloud_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace localization_common
{
class CloudSubscriber
{
public:
  CloudSubscriber(rclcpp::Node::SharedPtr node, std::string topic_name, size_t buff_size);
  void parse_data(std::deque<CloudData> & deque_cloud_data);

private:
  void msg_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;

  std::deque<CloudData> new_cloud_data_;
  std::mutex buff_mutex_;
};
}  // namespace localization_common
