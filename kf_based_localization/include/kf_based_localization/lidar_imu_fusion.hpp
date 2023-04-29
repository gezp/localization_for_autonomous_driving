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
#include "localization_common/sensor_data/pose_data.hpp"
#include "kf_based_localization/kalman_filter/eskf.hpp"

namespace kf_based_localization
{

class LidarImuFusion
{
public:
  LidarImuFusion();
  bool init_config(const YAML::Node & config_node);
  bool init(
    const Eigen::Matrix4f & init_pose, const Eigen::Vector3f & init_vel,
    const localization_common::IMUData & init_imu_data);
  bool process_imu_data(const localization_common::IMUData & imu_data);
  bool process_lidar_data(const localization_common::PoseData & lidar_pose_data);
  bool has_inited() const {return has_inited_;}
  double get_time() {return kalman_filter_->get_time();}
  localization_common::ImuNavState get_imu_nav_state();

private:
  std::shared_ptr<Eskf> kalman_filter_;
  bool has_inited_ = false;
  // data
  double gravity_magnitude_;
  Eigen::Matrix<double, 6, 1> lidar_pose_noise_;
  Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Vector3f current_vel_ = Eigen::Vector3f::Zero();
};

}  // namespace kf_based_localization
