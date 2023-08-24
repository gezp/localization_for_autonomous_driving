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

#include <yaml-cpp/yaml.h>

#include <deque>
#include <string>
#include <memory>
#include <vector>

#include "localization_common/sensor_data/imu_data.hpp"
#include "localization_common/sensor_data/odom_data.hpp"
#include "imu_odometry/imu_integration.hpp"
#include "graph_based_localization/graph_optimizer/ceres_graph_optimizer.hpp"

namespace graph_based_localization
{

class SlidingWindow
{
public:
  SlidingWindow();
  bool init_with_config(const YAML::Node & config_node);
  void set_extrinsic(const Eigen::Matrix4d & T_base_imu);
  bool add_imu_data(const localization_common::ImuData & imu);
  bool add_lidar_pose(const localization_common::OdomData & lidar_pose);
  bool add_gnss_pose(const localization_common::OdomData & gnss_pose);
  bool update();
  bool has_new_optimized();
  localization_common::ImuNavState get_imu_nav_state();
  localization_common::OdomData get_current_odom();

private:
  bool init_graph_optimizer(const YAML::Node & config_node);
  bool check_valid_lidar();
  bool check_new_key_frame(const localization_common::OdomData & odom);
  int create_graph_node(double time, const Eigen::Matrix4d & pose, const Eigen::Vector3d & vel);
  int create_graph_node_from_odom(const localization_common::OdomData & odom);
  int create_graph_node_from_lidar(size_t idx, size_t neighbor_idx);
  int create_graph_node_from_imu(const std::vector<localization_common::ImuData> & imus);
  bool get_synced_imu_buffer(double time, std::vector<localization_common::ImuData> & buffer);
  bool get_synced_gnss(double time, localization_common::OdomData & odom);
  bool try_init_from_lidar();
  bool update_graph();

private:
  // optimizer:
  std::shared_ptr<CeresGraphOptimizer> graph_optimizer_;
  // params
  struct
  {
    float max_distance;
    float max_interval;
  } key_frame_config_;
  bool use_gnss_ = true;
  bool use_lidar_pose_ = false;
  bool use_imu_pre_integration_ = false;
  double gravity_magnitude_;
  size_t sliding_window_size_;
  // measurement noise
  Eigen::VectorXd lidar_odometry_noise_;
  Eigen::VectorXd lidar_pose_noise_;
  Eigen::VectorXd gnss_position_noise_;
  // imu noise
  ImuConfig imu_config_;
  //
  Eigen::Matrix4d T_base_imu_ = Eigen::Matrix4d::Identity();
  // data
  std::deque<localization_common::ImuData> imu_buffer_;
  std::deque<localization_common::OdomData> lidar_pose_buffer_;
  std::deque<localization_common::OdomData> gnss_pose_buffer_;
  size_t unhandled_lidar_idx_{0};
  localization_common::ImuData current_imu_;
  localization_common::OdomData latest_key_frame_;
  bool has_new_optimized_ = false;
  bool has_inited_{false};
};

}  // namespace graph_based_localization
