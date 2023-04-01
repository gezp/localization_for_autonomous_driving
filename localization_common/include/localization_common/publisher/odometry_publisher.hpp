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
#include "nav_msgs/msg/odometry.hpp"
//
#include "localization_common/sensor_data/velocity_data.hpp"


namespace localization_common
{
class OdometryPublisher
{
public:
  OdometryPublisher(
    rclcpp::Node::SharedPtr node, std::string topic_name, std::string base_frame_id,
    std::string child_frame_id, int buff_size);
  OdometryPublisher() = default;
  void publish(
    const Eigen::Matrix4f & transform_matrix, const VelocityData & velocity_data,
    rclcpp::Time time);
  void publish(const Eigen::Matrix4f & transform_matrix, double time);
  void publish(const Eigen::Matrix4f & transform_matrix);
  void publish(
    const Eigen::Matrix4f & transform_matrix, const VelocityData & velocity_data, double time);
  void publish(const Eigen::Matrix4f & transform_matrix, const VelocityData & velocity_data);
  void publish(const Eigen::Matrix4f & transform_matrix, const Eigen::Vector3f & vel, double time);
  void publish(const Eigen::Matrix4f & transform_matrix, const Eigen::Vector3f & vel);
  bool has_subscribers() {return publisher_->get_subscription_count() > 0;}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  VelocityData velocity_data_;
  nav_msgs::msg::Odometry odometry_;
};
}  // namespace localization_common
