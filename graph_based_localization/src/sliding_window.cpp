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

#include "graph_based_localization/sliding_window.hpp"

#include "localization_common/sensor_data_utils.hpp"

namespace graph_based_localization
{

SlidingWindow::SlidingWindow() {}

bool SlidingWindow::init_with_config(const YAML::Node & config_node)
{
  // key frame selection config
  key_frame_config_.max_distance = config_node["key_frame"]["max_distance"].as<float>();
  key_frame_config_.max_interval = config_node["key_frame"]["max_interval"].as<float>();
  // constraints
  use_gnss_ = config_node["use_gnss"].as<bool>();
  use_lidar_pose_ = config_node["use_lidar_pose"].as<bool>();
  use_imu_pre_integration_ = config_node["use_imu_pre_integration"].as<bool>();
  sliding_window_size_ = config_node["sliding_window_size"].as<int>();
  // gravity
  gravity_magnitude_ = config_node["earth"]["gravity_magnitude"].as<double>();
  imu_config_.gravity = Eigen::Vector3d(0.0, 0.0, -gravity_magnitude_);
  // measurement noise
  lidar_odometry_noise_.resize(6);
  lidar_pose_noise_.resize(6);
  for (int i = 0; i < 6; ++i) {
    lidar_odometry_noise_(i) = config_node["measurement_noise"]["lidar_odometry"][i].as<double>();
    lidar_pose_noise_(i) = config_node["measurement_noise"]["lidar_pose"][i].as<double>();
  }
  gnss_position_noise_.resize(3);
  for (int i = 0; i < 3; i++) {
    gnss_position_noise_(i) = config_node["measurement_noise"]["gnss_position"][i].as<double>();
  }
  // imu noise
  imu_config_.accel_noise = config_node["imu_noise"]["accel"].as<double>();
  imu_config_.gyro_noise = config_node["imu_noise"]["gyro"].as<double>();
  imu_config_.accel_bias_noise = config_node["imu_noise"]["accel_bias"].as<double>();
  imu_config_.gyro_bias_noise = config_node["imu_noise"]["gyro_bias"].as<double>();
  // c. sliding window config:
  init_graph_optimizer(config_node);
  // print info
  std::cout << "SlidingWindow params:" << std::endl
            << "\timu_noise.accel: " << imu_config_.accel_noise << std::endl
            << "\timu_noise.gyro: " << imu_config_.gyro_noise << std::endl
            << "\timu_noise.accel_bias: " << imu_config_.accel_bias_noise << std::endl
            << "\timu_noise.gyro_bias: " << imu_config_.gyro_bias_noise << std::endl
            << std::endl;
  return true;
}

void SlidingWindow::set_extrinsic(const Eigen::Matrix4d & T_base_imu) {T_base_imu_ = T_base_imu;}

bool SlidingWindow::init_graph_optimizer(const YAML::Node & /*config_node*/)
{
  graph_optimizer_ = std::make_shared<CeresGraphOptimizer>(imu_config_);
  return true;
}

bool SlidingWindow::add_imu_data(const localization_common::ImuData & imu)
{
  imu_buffer_.push_back(imu);
  return true;
}

bool SlidingWindow::add_lidar_pose(const localization_common::OdomData & lidar_pose)
{
  lidar_pose_buffer_.push_back(lidar_pose);
  return true;
}
bool SlidingWindow::add_gnss_pose(const localization_common::OdomData & gnss_pose)
{
  gnss_pose_buffer_.push_back(gnss_pose);
  return true;
}

bool SlidingWindow::update()
{
  has_new_key_frame_ = false;
  has_new_optimized_ = false;
  if (!has_inited_) {
    // init graph
    return try_init();
  }
  if (lidar_pose_buffer_.empty() || gnss_pose_buffer_.empty()) {
    return false;
  }
  auto lidar_pose = lidar_pose_buffer_.front();
  auto gnss_pose = gnss_pose_buffer_.front();
  if (lidar_pose.time != gnss_pose.time) {
    std::cout << "lidar and gnss not sync" << std::endl;
    return false;
  }
  if (imu_buffer_.back().time < lidar_pose.time) {
    // large imu delay. wait imu
    return false;
  }
  current_lidar_pose_ = lidar_pose;
  current_gnss_pose_ = gnss_pose;
  lidar_pose_buffer_.pop_front();
  gnss_pose_buffer_.pop_front();
  if (imu_buffer_.front().time > lidar_pose.time) {
    // drop old data
    std::cout << "drop old data" << std::endl;
    return false;
  }
  if (!check_new_key_frame(lidar_pose)) {
    return false;
  }
  has_new_key_frame_ = true;
  latest_key_frame_ = lidar_pose;
  // update integrated imu buffer
  integrated_imu_buffer_.clear();
  integrated_imu_buffer_.push_back(current_imu_);
  localization_common::ImuData imu;
  while (imu_buffer_.front().time <= lidar_pose.time) {
    imu = imu_buffer_.front();
    imu_buffer_.pop_front();
    integrated_imu_buffer_.push_back(imu);
  }
  // get sync imu
  if (imu.time == lidar_pose.time) {
    current_imu_ = imu;
  } else {
    current_imu_ = localization_common::interpolate_imu(imu, imu_buffer_.front(), lidar_pose.time);
    integrated_imu_buffer_.push_back(current_imu_);
  }
  // update graph
  update_graph();
  return true;
}

bool SlidingWindow::has_new_key_frame() {return has_new_key_frame_;}

bool SlidingWindow::has_new_optimized() {return has_new_optimized_;}

localization_common::ImuNavState SlidingWindow::get_imu_nav_state()
{
  return graph_optimizer_->get_imu_nav_state();
}

localization_common::OdomData SlidingWindow::get_current_odom()
{
  auto nav_state = graph_optimizer_->get_imu_nav_state();
  // odometry for imu frame
  localization_common::OdomData odom;
  odom.time = nav_state.time;
  odom.pose.block<3, 1>(0, 3) = nav_state.position;
  odom.pose.block<3, 3>(0, 0) = nav_state.orientation;
  odom.linear_velocity = nav_state.linear_velocity;
  odom.angular_velocity = current_imu_.angular_velocity;
  // odometry for base frame
  return localization_common::transform_odom(odom, T_base_imu_.inverse());
}

bool SlidingWindow::check_new_key_frame(const localization_common::OdomData & odom)
{
  bool has_new_key_frame = false;
  // key frame selection for sliding window:
  auto distance =
    (odom.pose.block<3, 1>(0, 3) - latest_key_frame_.pose.block<3, 1>(0, 3)).lpNorm<1>();
  if (
    distance > key_frame_config_.max_distance ||
    (odom.time - latest_key_frame_.time) > key_frame_config_.max_interval)
  {
    has_new_key_frame = true;
  } else {
    has_new_key_frame = false;
  }
  return has_new_key_frame;
}

bool SlidingWindow::try_init()
{
  if (lidar_pose_buffer_.empty() || gnss_pose_buffer_.empty()) {
    return false;
  }
  auto lidar_pose = lidar_pose_buffer_.front();
  auto gnss_pose = gnss_pose_buffer_.front();
  if (lidar_pose.time != gnss_pose.time) {
    std::cout << "lidar and gnss not sync" << std::endl;
    return false;
  }
  if (imu_buffer_.back().time < lidar_pose.time) {
    // large imu delay. wait imu
    return false;
  }
  current_lidar_pose_ = lidar_pose;
  current_gnss_pose_ = gnss_pose;
  lidar_pose_buffer_.pop_front();
  gnss_pose_buffer_.pop_front();
  if (imu_buffer_.front().time > lidar_pose.time) {
    // drop old data
    std::cout << "drop old data" << std::endl;
    return false;
  }
  has_new_key_frame_ = true;
  latest_key_frame_ = lidar_pose;
  //
  localization_common::ImuData imu;
  while (imu_buffer_.front().time <= lidar_pose.time) {
    imu = imu_buffer_.front();
    imu_buffer_.pop_front();
  }
  if (imu.time == lidar_pose.time) {
    current_imu_ = imu;
  } else {
    current_imu_ = localization_common::interpolate_imu(imu, imu_buffer_.front(), lidar_pose.time);
  }
  update_graph();
  has_inited_ = true;
  return true;
}

bool SlidingWindow::update_graph()
{
  // initial state
  auto odom = transform_odom(current_gnss_pose_, T_base_imu_);
  localization_common::ImuNavState imu_nav_state;
  imu_nav_state.time = odom.time;
  imu_nav_state.position = odom.pose.block<3, 1>(0, 3);
  imu_nav_state.orientation = odom.pose.block<3, 3>(0, 0);
  imu_nav_state.linear_velocity = imu_nav_state.orientation * odom.linear_velocity;
  if (graph_optimizer_->get_vertex_count() > 0) {
    auto last_state = graph_optimizer_->get_imu_nav_state();
    imu_nav_state.accel_bias = last_state.accel_bias;
    imu_nav_state.gyro_bias = last_state.gyro_bias;
  }
  // add vertex
  int vertex_idx = graph_optimizer_->add_vertex(imu_nav_state, false);
  // lidar map pose constraint
  if (use_lidar_pose_) {
    Eigen::Matrix4d prior_pose = current_lidar_pose_.pose * T_base_imu_;
    graph_optimizer_->add_absolute_pose_edge(vertex_idx, prior_pose, lidar_pose_noise_);
  }
  // imu pre integration constraint
  if (vertex_idx > 0 && use_imu_pre_integration_) {
    std::cout << "integrated_imu_buffer:" << integrated_imu_buffer_.size() << std::endl;
    graph_optimizer_->add_imu_pre_integration_edge(
      vertex_idx - 1, vertex_idx, integrated_imu_buffer_);
  }
  if (graph_optimizer_->optimize()) {
    if (graph_optimizer_->get_vertex_count() > sliding_window_size_) {
      graph_optimizer_->marginalize(1);
    }
    has_new_optimized_ = true;
  }
  return true;
}

}  // namespace graph_based_localization
