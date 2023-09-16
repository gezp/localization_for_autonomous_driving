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

#include <thread>
#include <memory>
#include <deque>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
// subscriber
#include "localization_common/subscriber/cloud_subscriber.hpp"
#include "localization_common/subscriber/nav_sat_fix_subscriber.hpp"
#include "localization_common/subscriber/imu_subscriber.hpp"
#include "localization_common/subscriber/twist_subscriber.hpp"
// publisher
#include "localization_common/publisher/cloud_publisher.hpp"
#include "localization_common/publisher/imu_publisher.hpp"
#include "localization_common/publisher/odometry_publisher.hpp"
#include "localization_common/publisher/pos_vel_publisher.hpp"
//
#include "localization_common/distortion_adjust.hpp"
#include "localization_common/extrinsics_manager.hpp"
#include "localization_common/msg_util.hpp"

namespace localization_common
{
class KittiPreprocessNode
{
public:
  explicit KittiPreprocessNode(rclcpp::Node::SharedPtr node);

private:
  bool run();
  bool read_data();
  bool has_data();
  bool valid_data();
  bool transform_data();
  bool publish_data();

private:
  // subscriber
  std::shared_ptr<CloudSubscriber<pcl::PointXYZ>> cloud_sub_;
  std::shared_ptr<ImuSubscriber> imu_sub_;
  std::shared_ptr<TwistSubscriber> twist_sub_;
  std::shared_ptr<NavSatFixSubscriber> nav_sat_fix_sub_;
  // publisher
  std::shared_ptr<CloudPublisher<pcl::PointXYZ>> cloud_pub_;
  std::shared_ptr<OdometryPublisher> gnss_pose_pub_;
  std::shared_ptr<ImuPublisher> imu_pub_;
  std::shared_ptr<PosVelPublisher> pos_vel_pub_;
  // models
  std::shared_ptr<DistortionAdjust> distortion_adjust_;
  // tf
  std::shared_ptr<ExtrinsicsManager> extrinsics_manager_;
  std::string imu_frame_id_{"imu"};
  std::string lidar_frame_id_{"lidar"};
  std::string base_frame_id_{"base"};
  Eigen::Matrix4d T_imu_base_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_base_lidar_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_imu_lidar_ = Eigen::Matrix4d::Identity();
  bool is_valid_extrinsics_{false};
  // gnss datum (latitude, longitude, altitude)
  bool use_manual_map_origin_{true};
  std::vector<double> map_origin_{48.982545, 8.390366, 116.382141};
  // data
  std::deque<LidarData<pcl::PointXYZ>> lidar_data_buff_;
  std::deque<ImuData2> imu_data_buff_;
  std::deque<TwistData> twist_data_buff_;
  std::deque<GnssData> gnss_data_buff_;

  LidarData<pcl::PointXYZ> current_lidar_data_;
  ImuData2 current_imu_data_;
  TwistData current_twist_data_;
  GnssData current_gnss_data_;

  PosVelData pos_vel_;
  Eigen::Matrix4d gnss_pose_ = Eigen::Matrix4d::Identity();
  //
  std::unique_ptr<std::thread> run_thread_;
  bool run_flag_{true};
};
}  // namespace localization_common
