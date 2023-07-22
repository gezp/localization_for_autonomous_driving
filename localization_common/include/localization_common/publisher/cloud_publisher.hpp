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

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace localization_common
{
template<typename PointT>
class CloudPublisher
{
  using PointCloudPtr = typename pcl::PointCloud<PointT>::Ptr;

public:
  CloudPublisher(
    rclcpp::Node::SharedPtr node, std::string topic_name, std::string frame_id, size_t buff_size)
  : node_(node), frame_id_(frame_id)
  {
    publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, buff_size);
  }
  void publish(PointCloudPtr cloud, rclcpp::Time time)
  {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = time;
    msg.header.frame_id = frame_id_;
    publisher_->publish(msg);
  }
  void publish(PointCloudPtr cloud, double time)
  {
    rclcpp::Time ros_time(static_cast<uint64_t>(time * 1e9));
    publish(cloud, ros_time);
  }
  void publish(PointCloudPtr cloud) {publish(cloud, node_->get_clock()->now());}

  bool has_subscribers() {return publisher_->get_subscription_count() > 0;}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::string frame_id_;
};
}  // namespace localization_common
