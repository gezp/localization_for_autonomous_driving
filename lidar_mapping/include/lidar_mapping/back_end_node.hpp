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
#include <string>
//
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
//
#include "localization_interfaces/srv/optimize_map.hpp"
#include "localization_interfaces/srv/save_map.hpp"
#include "localization_common/subscriber/cloud_subscriber.hpp"
#include "localization_common/subscriber/loop_candidate_subscriber.hpp"
#include "localization_common/subscriber/odometry_subscriber.hpp"
#include "localization_common/publisher/cloud_publisher.hpp"
#include "localization_common/publisher/lidar_frames_publisher.hpp"
#include "localization_common/publisher/path_publisher.hpp"
#include "localization_common/publisher/odometry_publisher.hpp"
#include "localization_common/extrinsics_manager.hpp"
#include "lidar_mapping/back_end.hpp"

namespace lidar_mapping
{
class BackEndNode
{
public:
  explicit BackEndNode(rclcpp::Node::SharedPtr node);
  ~BackEndNode();

private:
  bool run();
  bool force_optimize();
  bool read_data();
  bool has_data();
  bool valid_data();
  bool publish_data();

private:
  // sub
  std::shared_ptr<localization_common::CloudSubscriber> cloud_sub_;
  std::shared_ptr<localization_common::OdometrySubscriber> gnss_pose_sub_;
  std::shared_ptr<localization_common::OdometrySubscriber> lidar_odom_sub_;
  std::shared_ptr<localization_common::LoopCandidateSubscriber> loop_candidate_sub_;
  // pub
  std::shared_ptr<localization_common::LidarFramesPublisher> key_frames_pub_;
  std::shared_ptr<localization_common::PathPublisher> optimized_path_pub_;
  std::shared_ptr<localization_common::OdometryPublisher> optimized_odom_pub_;
  std::shared_ptr<localization_common::CloudPublisher> global_map_pub_;
  // tf
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  std::shared_ptr<localization_common::ExtrinsicsManager> extrinsics_manager_;
  std::string lidar_frame_id_{"lidar"};
  std::string base_frame_id_{"base"};
  Eigen::Matrix4d T_base_lidar_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_lidar_base_ = Eigen::Matrix4d::Identity();
  bool is_valid_extrinsics_{false};
  bool publish_tf_{false};
  // srv
  rclcpp::Service<localization_interfaces::srv::OptimizeMap>::SharedPtr optimize_map_srv_;
  bool need_optimize_map_{false};
  rclcpp::Service<localization_interfaces::srv::SaveMap>::SharedPtr save_map_srv_;
  bool save_map_flag_{false};
  // back end and flow thread
  std::shared_ptr<BackEnd> back_end_;
  std::unique_ptr<std::thread> run_thread_;
  bool exit_{false};
  // data
  std::deque<localization_common::LidarData<pcl::PointXYZ>> lidar_data_buff_;
  std::deque<localization_common::OdomData> gnss_pose_data_buff_;
  std::deque<localization_common::OdomData> lidar_odom_data_buff_;
  std::deque<localization_common::LoopCandidate> loop_candidate_data_buff_;

  localization_common::OdomData current_lidar_odom_data_;
  localization_common::LidarData<pcl::PointXYZ> current_lidar_data_;
};
}  // namespace lidar_mapping
