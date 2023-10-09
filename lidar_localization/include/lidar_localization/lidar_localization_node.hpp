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
#include "localization_common/subscriber/gnss_subscriber.hpp"
#include "localization_common/subscriber/odometry_subscriber.hpp"
#include "localization_common/publisher/cloud_publisher.hpp"
#include "localization_common/publisher/odometry_publisher.hpp"
#include "localization_common/extrinsics_manager.hpp"
#include "lidar_localization/lidar_localization.hpp"

namespace lidar_localization
{
class LidarLocalizationNode
{
public:
  explicit LidarLocalizationNode(rclcpp::Node::SharedPtr node);
  ~LidarLocalizationNode();

private:
  bool run();
  bool read_data();
  bool publish_data();

private:
  // subscriber
  std::shared_ptr<localization_common::CloudSubscriber<pcl::PointXYZ>> cloud_sub_;
  std::shared_ptr<localization_common::GnssSubscriber> gnss_data_sub_;
  std::shared_ptr<localization_common::OdometrySubscriber> gnss_odom_sub_;
  // publisher
  std::shared_ptr<localization_common::CloudPublisher> global_map_pub_;
  std::shared_ptr<localization_common::CloudPublisher> local_map_pub_;
  std::shared_ptr<localization_common::CloudPublisher> current_scan_pub_;
  std::shared_ptr<localization_common::OdometryPublisher> lidar_pose_pub_;
  // tf
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  std::shared_ptr<localization_common::ExtrinsicsManager> extrinsics_manager_;
  std::string lidar_frame_id_{"lidar"};
  std::string base_frame_id_{"base"};
  Eigen::Matrix4d T_base_lidar_ = Eigen::Matrix4d::Identity();
  bool is_valid_extrinsics_{false};
  bool publish_tf_{false};
  // lidar_localization and process thread
  std::shared_ptr<LidarLocalization> lidar_localization_;
  std::unique_ptr<std::thread> run_thread_;
  bool exit_{false};
  // data
  std::deque<localization_common::LidarData<pcl::PointXYZ>> lidar_data_buffer_;
  std::deque<localization_common::GnssData> gnss_data_buffer_;
  std::deque<localization_common::OdomData> gnss_odom_buffer_;
};
}  // namespace lidar_localization
