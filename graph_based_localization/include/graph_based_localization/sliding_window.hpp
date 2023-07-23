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
#include <fstream>
#include <string>
#include <memory>
#include <vector>

#include "localization_common/sensor_data/imu_data.hpp"
#include "localization_common/sensor_data/key_frame.hpp"
#include "localization_common/sensor_data/odom_data.hpp"
#include "graph_based_localization/graph_optimizer/ceres_graph_optimizer.hpp"

namespace graph_based_localization
{

class SlidingWindow
{
public:
  SlidingWindow();
  bool init_with_config(const YAML::Node & config_node);
  void set_extrinsic(const Eigen::Matrix4f & T_lidar_imu);
  bool add_raw_imu(const localization_common::ImuData & imu_data);
  bool update(
    const localization_common::OdomData & lidar_pose, const localization_common::ImuData & imu_data,
    const localization_common::OdomData & gnss_pose);
  bool has_new_key_frame();
  bool has_new_optimized();
  localization_common::ImuNavState get_imu_nav_state();

private:
  bool init_graph_optimizer(const YAML::Node & config_node);
  bool check_new_key_frame(const localization_common::OdomData & odom);
  bool update_graph();

private:
  bool has_new_key_frame_ = false;
  bool has_new_optimized_ = false;
  //
  Eigen::Matrix4f T_lidar_imu_ = Eigen::Matrix4f::Identity();

  localization_common::KeyFrame current_key_frame_;
  localization_common::OdomData current_lidar_pose_;
  localization_common::OdomData current_gnss_pose_;
  localization_common::KeyFrame last_key_frame_;
  // key frame buffer:
  std::deque<localization_common::KeyFrame> key_frames_;
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
  // gravity
  double gravity_magnitude_;
  //
  size_t sliding_window_size_;
  // measurement noise
  Eigen::VectorXd lidar_odometry_noise_;
  Eigen::VectorXd lidar_pose_noise_;
  Eigen::VectorXd gnss_position_noise_;
  // imu noise
  ImuConfig imu_config_;
  std::vector<localization_common::ImuData> imu_buffer_;
};

}  // namespace graph_based_localization
