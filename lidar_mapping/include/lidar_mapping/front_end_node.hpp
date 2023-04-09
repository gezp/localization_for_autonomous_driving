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
#include <deque>
#include <fstream>
#include <string>
//
#include "rclcpp/rclcpp.hpp"
#include "localization_common/subscriber/cloud_subscriber.hpp"
#include "localization_common/publisher/odometry_publisher.hpp"
#include "lidar_mapping/front_end.hpp"

namespace lidar_mapping
{
class FrontEndNode
{
public:
  explicit FrontEndNode(rclcpp::Node::SharedPtr node);
  ~FrontEndNode();

private:
  bool run();
  bool read_data();
  bool has_data();
  bool valid_data();
  bool update_odometry();
  bool publish_data();

private:
  // sub & pub
  std::shared_ptr<localization_common::CloudSubscriber> cloud_sub_;
  std::shared_ptr<localization_common::OdometryPublisher> lidar_odom_pub_;
  // front end and flow thread
  std::shared_ptr<FrontEnd> front_end_;
  std::unique_ptr<std::thread> run_thread_;
  bool exit_{false};
  // data
  std::deque<localization_common::CloudData> cloud_data_buff_;
  localization_common::CloudData current_cloud_data_;
  Eigen::Matrix4f lidar_odometry_ = Eigen::Matrix4f::Identity();
};
}  // namespace lidar_mapping
