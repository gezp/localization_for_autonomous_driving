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

#include "kf_based_localization/lidar_imu_fusion.hpp"

#include <iostream>

namespace kf_based_localization
{

LidarImuFusion::LidarImuFusion() {}

bool LidarImuFusion::init_config(const YAML::Node & config_node)
{
  // gravity
  gravity_magnitude_ = config_node["earth"]["gravity_magnitude"].as<double>();
  // lidar pose noise
  auto pos_noise = config_node["measurement_noise"]["lidar_pose"]["pos"].as<double>();
  auto ori_noise = config_node["measurement_noise"]["lidar_pose"]["ori"].as<double>();
  lidar_pose_noise_ << pos_noise, pos_noise, pos_noise, ori_noise, ori_noise, ori_noise;
  // set up fusion method:
  auto fusion_method = config_node["fusion_method"].as<std::string>();
  if (fusion_method == "eskf") {
    kalman_filter_ = std::make_shared<Eskf>(config_node[fusion_method]);
  } else {
    std::cerr << "Fusion method " << fusion_method << " NOT FOUND!";
    return false;
  }
  std::cout << "Localization Fusion Method: " << fusion_method << std::endl;
  kalman_filter_->print_info();
  return true;
}

bool LidarImuFusion::init(
  const Eigen::Matrix4f & init_pose, const Eigen::Vector3f & init_vel,
  const localization_common::ImuData & init_imu_data)
{
  localization_common::ImuNavState init_nav_state;
  init_nav_state.time = init_imu_data.time;
  init_nav_state.position = init_pose.block<3, 1>(0, 3).cast<double>();
  init_nav_state.orientation = init_pose.block<3, 3>(0, 0).cast<double>();
  init_nav_state.linear_velocity = init_vel.cast<double>();
  init_nav_state.gravity = Eigen::Vector3d(0.0, 0.0, gravity_magnitude_);
  init_nav_state.gyro_bias = Eigen::Vector3d::Zero();
  init_nav_state.accel_bias = Eigen::Vector3d::Zero();
  kalman_filter_->init_state(init_nav_state, init_imu_data);
  has_inited_ = true;
  return true;
}

bool LidarImuFusion::process_imu_data(const localization_common::ImuData & imu_data)
{
  return kalman_filter_->predict(imu_data);
}

bool LidarImuFusion::process_lidar_data(const localization_common::PoseData & lidar_pose_data)
{
  Eigen::Matrix4d lidar_pose = lidar_pose_data.pose.cast<double>();
  return kalman_filter_->observe_pose(lidar_pose, lidar_pose_noise_);
}

localization_common::ImuNavState LidarImuFusion::get_imu_nav_state()
{
  return kalman_filter_->get_imu_nav_state();
}

}  // namespace kf_based_localization
