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
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
// subscriber
#include "localization_common/subscriber/cloud_subscriber.hpp"
#include "localization_common/subscriber/gnss_subscriber.hpp"
#include "localization_common/subscriber/imu_subscriber.hpp"
#include "localization_common/subscriber/velocity_subscriber.hpp"
// publisher
#include "localization_common/publisher/cloud_publisher.hpp"
#include "localization_common/publisher/imu_publisher.hpp"
#include "localization_common/publisher/odometry_publisher.hpp"
#include "localization_common/publisher/pos_vel_publisher.hpp"
//
#include "localization_common/distortion_adjust.hpp"
#include "localization_common/tf_utils.hpp"

namespace localization_common
{
class KittiPreprocessNode
{
public:
  explicit KittiPreprocessNode(rclcpp::Node::SharedPtr node);

private:
  bool run();
  bool read_data();
  bool init_calibration();
  bool has_data();
  bool valid_data();
  bool transform_data();
  bool publish_data();

private:
  // subscriber
  std::shared_ptr<CloudSubscriber> cloud_sub_;
  std::shared_ptr<IMUSubscriber> imu_sub_;
  std::shared_ptr<VelocitySubscriber> velocity_sub_;
  std::shared_ptr<GNSSSubscriber> gnss_sub_;
  // publisher
  std::shared_ptr<CloudPublisher> cloud_pub_;
  std::shared_ptr<OdometryPublisher> gnss_pose_pub_;
  std::shared_ptr<IMUPublisher> imu_pub_;
  std::shared_ptr<PosVelPublisher> pos_vel_pub_;
  // models
  std::shared_ptr<DistortionAdjust> distortion_adjust_;
  // tf
  std::string imu_frame_id_{"imu"};
  std::string lidar_frame_id_{"lidar"};
  std::string base_link_frame_id_{"base_link"};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  Eigen::Matrix4f base_link_to_imu_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f base_link_to_lidar_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();
  // gnss datum (latitude, longitude, altitude)
  bool use_manual_gnss_datum_{false};
  std::vector<double> gnss_datum_{48.982545, 8.390366, 116.382141};
  // data
  std::deque<CloudData> cloud_data_buff_;
  std::deque<IMUData> imu_data_buff_;
  std::deque<VelocityData> velocity_data_buff_;
  std::deque<GNSSData> gnss_data_buff_;

  CloudData current_cloud_data_;
  IMUData current_imu_data_;
  VelocityData current_velocity_data_;
  GNSSData current_gnss_data_;

  PosVelData pos_vel_;
  Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();
  //
  std::unique_ptr<std::thread> run_thread_;
  bool run_flag_{true};
};
}  // namespace localization_common
