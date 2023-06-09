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
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
//
#include "localization_interfaces/srv/save_odometry.hpp"
#include "localization_common/subscriber/cloud_subscriber.hpp"
#include "localization_common/subscriber/odometry_subscriber.hpp"
#include "localization_common/publisher/cloud_publisher.hpp"
#include "localization_common/publisher/odometry_publisher.hpp"
#include "lidar_localization/matching.hpp"

namespace lidar_localization
{
class MatchingNode
{
public:
  explicit MatchingNode(rclcpp::Node::SharedPtr node);
  ~MatchingNode();

private:
  bool run();
  bool read_data();
  bool has_data();
  bool valid_data();
  bool update_matching();
  bool publish_data();

private:
  // subscriber
  std::shared_ptr<localization_common::CloudSubscriber> cloud_sub_;
  std::shared_ptr<localization_common::OdometrySubscriber> gnss_sub_;
  // publisher
  std::shared_ptr<localization_common::CloudPublisher> global_map_pub_;
  std::shared_ptr<localization_common::CloudPublisher> local_map_pub_;
  std::shared_ptr<localization_common::CloudPublisher> current_scan_pub_;
  std::shared_ptr<localization_common::OdometryPublisher> lidar_odom_pub_;
  // tf
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  std::string base_link_frame_id_{"base_link"};
  bool publish_tf_{false};
  // matching and process thread
  std::shared_ptr<Matching> matching_;
  std::unique_ptr<std::thread> run_thread_;
  bool exit_{false};
  // data
  std::deque<localization_common::CloudData> cloud_data_buff_;
  std::deque<localization_common::PoseData> gnss_data_buff_;
  localization_common::CloudData current_cloud_data_;
  localization_common::PoseData current_gnss_data_;
  Eigen::Matrix4f lidar_odometry_ = Eigen::Matrix4f::Identity();
};
}  // namespace lidar_localization
