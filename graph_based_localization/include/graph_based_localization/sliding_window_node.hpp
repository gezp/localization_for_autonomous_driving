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
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "localization_common/subscriber/odometry_subscriber.hpp"
#include "localization_common/publisher/odometry_publisher.hpp"
#include "localization_common/subscriber/imu_subscriber.hpp"
#include "localization_common/tf_utils.hpp"
#include "graph_based_localization/sliding_window.hpp"

namespace graph_based_localization
{

class SlidingWindowNode
{
public:
  explicit SlidingWindowNode(rclcpp::Node::SharedPtr node);
  ~SlidingWindowNode();

private:
  bool run();
  bool read_data();
  bool has_data();
  bool valid_data();
  bool init_calibration();
  bool update_back_end();
  bool publish_data();

private:
  // sub&pub
  std::shared_ptr<localization_common::OdometrySubscriber> lidar_pose_sub_;
  std::shared_ptr<localization_common::OdometrySubscriber> gnss_pose_sub_;
  std::shared_ptr<localization_common::IMUSubscriber> imu_raw_sub_;
  std::shared_ptr<localization_common::IMUSubscriber> imu_synced_sub_;
  std::shared_ptr<localization_common::OdometryPublisher> optimized_odom_pub_;
  // tf
  std::string imu_frame_id_{"imu"};
  std::string base_link_frame_id_{"base_link"};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  Eigen::Matrix4f base_link_to_imu_ = Eigen::Matrix4f::Identity();
  // sliding window
  std::shared_ptr<SlidingWindow> sliding_window_;
  std::unique_ptr<std::thread> run_thread_;
  bool exit_{false};
  // synced data:
  std::deque<localization_common::PoseData> lidar_pose_data_buff_;
  std::deque<localization_common::PoseData> gnss_pose_data_buff_;
  std::deque<localization_common::IMUData> imu_raw_data_buff_;
  std::deque<localization_common::IMUData> imu_synced_data_buff_;
  localization_common::PoseData current_lidar_pose_data_;
  localization_common::PoseData current_gnss_pose_data_;
  localization_common::IMUData current_imu_data_;
};

}  // namespace graph_based_localization
