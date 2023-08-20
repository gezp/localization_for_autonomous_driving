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

#include "localization_common/sensor_data_utils.hpp"

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

void LidarImuFusion::set_extrinsic(const Eigen::Matrix4d & T_base_imu) {T_base_imu_ = T_base_imu;}

bool LidarImuFusion::init(
  const Eigen::Matrix4d & initial_pose, const Eigen::Vector3d & initial_vel,
  const localization_common::ImuData & init_imu_data)
{
  localization_common::ImuNavState init_nav_state;
  init_nav_state.time = init_imu_data.time;
  init_nav_state.position = initial_pose.block<3, 1>(0, 3);
  init_nav_state.orientation = initial_pose.block<3, 3>(0, 0);
  init_nav_state.linear_velocity = initial_vel;
  init_nav_state.gravity = Eigen::Vector3d(0.0, 0.0, gravity_magnitude_);
  init_nav_state.gyro_bias = Eigen::Vector3d::Zero();
  init_nav_state.accel_bias = Eigen::Vector3d::Zero();
  kalman_filter_->init_state(init_nav_state, init_imu_data);
  current_imu_ = init_imu_data;
  latest_correct_time_ = init_imu_data.time;
  has_inited_ = true;
  return true;
}

bool LidarImuFusion::add_imu_data(const localization_common::ImuData & imu)
{
  if (!has_inited_) {
    return false;
  }
  // check time
  if (imu.time < kalman_filter_->get_time()) {
    std::cout << "IMU has a older timestamp. Skip." << imu.time << std::endl;
    return false;
  }
  // put into buffer
  raw_imu_buffer_.push_back(imu);
  // predict range
  // TODO(gezp): use dynamic range, avoid to drop data.
  double predict_dt = latest_correct_time_ + 0.09;
  if (raw_imu_buffer_.front().time > predict_dt) {
    return false;
  }
  auto current_imu = raw_imu_buffer_.front();
  raw_imu_buffer_.pop_front();
  if (!kalman_filter_->predict(current_imu)) {
    std::cout << "predict failed." << std::endl;
    return false;
  }
  return true;
}

bool LidarImuFusion::add_observation_data(const localization_common::OdomData & lidar_odom)
{
  if (!has_inited_) {
    return false;
  }
  // check time
  if (lidar_odom.time < kalman_filter_->get_time()) {
    std::cout << "Lidar Observation has a older timestamp. Skip." << lidar_odom.time << std::endl;
  }
  if (raw_imu_buffer_.empty()) {
    return false;
  }
  if (lidar_odom.time > raw_imu_buffer_.back().time) {
    // large imu delay
    std::cout << "all imu is before lidar(imu delay), drop lidar data" << lidar_odom.time
              << std::endl;
    return false;
  }
  if (lidar_odom.time < raw_imu_buffer_.front().time) {
    // old lidar data
    std::cout << "drop older lidar data: " << lidar_odom.time << std::endl;
    return false;
  }
  // move filter forward
  localization_common::ImuData imu;
  while (raw_imu_buffer_.front().time < lidar_odom.time) {
    imu = raw_imu_buffer_.front();
    raw_imu_buffer_.pop_front();
    kalman_filter_->predict(imu);
    // TODO(gezp): need publish ?
  }
  auto & imu_after = raw_imu_buffer_.front();
  current_imu_ = localization_common::interpolate_imu(imu, imu_after, lidar_odom.time);
  kalman_filter_->predict(current_imu_);
  // correct
  Eigen::Matrix4d pose = lidar_odom.pose * T_base_imu_;
  if (!kalman_filter_->observe_pose(pose, lidar_pose_noise_)) {
    std::cout << "correct[lidar] failed." << std::endl;
    return false;
  }
  latest_correct_time_ = lidar_odom.time;
  return true;
}

localization_common::ImuNavState LidarImuFusion::get_imu_nav_state()
{
  return kalman_filter_->get_imu_nav_state();
}

localization_common::OdomData LidarImuFusion::get_current_odom()
{
  auto nav_state = kalman_filter_->get_imu_nav_state();
  // odometry for imu frame
  localization_common::OdomData odom_imu;
  odom_imu.time = nav_state.time;
  odom_imu.pose.block<3, 1>(0, 3) = nav_state.position;
  odom_imu.pose.block<3, 3>(0, 0) = nav_state.orientation;
  odom_imu.linear_velocity = nav_state.linear_velocity;
  odom_imu.angular_velocity = current_imu_.angular_velocity;
  // odometry for base frame
  return localization_common::transform_odom(odom_imu, T_base_imu_.inverse());
}

}  // namespace kf_based_localization
