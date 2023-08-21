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

#include <deque>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
//
#include "localization_common/subscriber/imu_subscriber.hpp"
#include "localization_common/subscriber/odometry_subscriber.hpp"
#include "localization_common/publisher/odometry_publisher.hpp"
#include "localization_common/extrinsics_manager.hpp"
#include "localization_common/msg_util.hpp"
#include "kf_based_localization/lidar_imu_fusion.hpp"

namespace kf_based_localization
{

class LidarImuFusionNode
{
public:
  explicit LidarImuFusionNode(rclcpp::Node::SharedPtr node);
  ~LidarImuFusionNode();

private:
  bool run();
  bool read_data();
  bool has_imu_data();
  bool has_lidar_data();
  bool valid_lidar_data();
  bool publish_fusion_odom();

private:
  // sub&pub
  std::shared_ptr<localization_common::ImuSubscriber> raw_imu_sub_;
  std::shared_ptr<localization_common::OdometrySubscriber> lidar_pose_sub_;
  std::shared_ptr<localization_common::OdometrySubscriber> gnss_pose_sub_;
  std::shared_ptr<localization_common::OdometryPublisher> fused_odom_pub_;
  // tf
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  std::shared_ptr<localization_common::ExtrinsicsManager> extrinsics_manager_;
  std::string imu_frame_id_{"imu"};
  std::string base_frame_id_{"base"};
  Eigen::Matrix4d T_base_imu_ = Eigen::Matrix4d::Identity();
  bool is_valid_extrinsics_{false};
  // fusion proccess flow
  std::shared_ptr<LidarImuFusion> fusion_;
  std::unique_ptr<std::thread> run_thread_;
  bool exit_{false};
  // data
  std::deque<localization_common::ImuData> imu_buffer_;
  std::deque<localization_common::OdomData> lidar_pose_buffer_;
  std::deque<localization_common::OdomData> gnss_pose_buffer_;
  localization_common::OdomData current_gnss_pose_;
  localization_common::OdomData current_lidar_pose_;
};

}  // namespace kf_based_localization
