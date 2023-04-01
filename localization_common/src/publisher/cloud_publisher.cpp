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

#include "localization_common/publisher/cloud_publisher.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace localization_common
{
CloudPublisher::CloudPublisher(
  rclcpp::Node::SharedPtr node, std::string topic_name, std::string frame_id, size_t buff_size)
: node_(node), frame_id_(frame_id)
{
  publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, buff_size);
}

void CloudPublisher::publish(PointXYZCloudPtr cloud_input, rclcpp::Time time)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*cloud_input, msg);
  msg.header.stamp = time;
  msg.header.frame_id = frame_id_;
  publisher_->publish(msg);
}

void CloudPublisher::publish(PointXYZCloudPtr cloud_input, double time)
{
  rclcpp::Time ros_time(static_cast<uint64_t>(time * 1e9));
  publish(cloud_input, ros_time);
}

void CloudPublisher::publish(PointXYZCloudPtr cloud_input)
{
  publish(cloud_input, node_->get_clock()->now());
}

}  // namespace localization_common
