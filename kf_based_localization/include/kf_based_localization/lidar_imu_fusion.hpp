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

#include <Eigen/Dense>
#include <deque>
#include <memory>

#include "localization_common/sensor_data/imu_data.hpp"
#include "localization_common/sensor_data/pos_vel_data.hpp"
#include "localization_common/sensor_data/odom_data.hpp"
#include "kf_based_localization/kalman_filter/eskf.hpp"

namespace kf_based_localization
{

class LidarImuFusion
{
public:
  LidarImuFusion();
  bool init_config(const YAML::Node & config_node);
  void set_extrinsic(const Eigen::Matrix4d & T_base_imu);
  bool add_imu_data(const localization_common::ImuData & imu);
  bool add_lidar_data(const localization_common::OdomData & lidar_pose);
  bool add_gnss_data(const localization_common::OdomData & gnss_pose);
  bool update();
  localization_common::ImuNavState get_imu_nav_state();
  localization_common::OdomData get_current_odom();

private:
  bool init_filter(
    const Eigen::Matrix4d & initial_pose, const Eigen::Vector3d & initial_velocity,
    const localization_common::ImuData & initial_imu);
  bool try_init_by_gnss();
  bool try_init_by_lidar();
  bool process_imu_data(const localization_common::ImuData & imu);
  bool process_observation_data(const localization_common::OdomData & lidar_pose);

private:
  std::shared_ptr<Eskf> kalman_filter_;
  // config
  double gravity_magnitude_;
  Eigen::Matrix<double, 6, 1> lidar_pose_noise_;
  Eigen::Matrix4d T_base_imu_ = Eigen::Matrix4d::Identity();
  // data
  std::deque<localization_common::ImuData> imu_buffer_;
  std::deque<localization_common::OdomData> lidar_buffer_;
  std::deque<localization_common::OdomData> gnss_buffer_;
  localization_common::ImuData current_imu_;
  double latest_correct_time_;
  bool has_inited_ = false;
};

}  // namespace kf_based_localization
