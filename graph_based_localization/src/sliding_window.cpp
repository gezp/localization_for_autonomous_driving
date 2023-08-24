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
  if (!imu_buffer_.empty() && imu.time <= imu_buffer_.back().time) {
    std::cout << "receive a early imu data" << std::endl;
    return false;
  }
  imu_buffer_.push_back(imu);
  if (imu_buffer_.size() > 1000) {
    imu_buffer_.pop_front();
  }
  return true;
}

bool SlidingWindow::add_lidar_pose(const localization_common::OdomData & lidar_pose)
{
  if (!lidar_pose_buffer_.empty() && lidar_pose.time <= lidar_pose_buffer_.back().time) {
    std::cout << "receive a early lidar data" << std::endl;
    return false;
  }
  lidar_pose_buffer_.push_back(lidar_pose);
  // at least keep one history
  if (lidar_pose_buffer_.size() > 100 && unhandled_lidar_idx_ > 1) {
    lidar_pose_buffer_.pop_front();
    unhandled_lidar_idx_--;
  }
  return true;
}

bool SlidingWindow::add_gnss_pose(const localization_common::OdomData & gnss_pose)
{
  if (!gnss_pose_buffer_.empty() && gnss_pose.time <= gnss_pose_buffer_.back().time) {
    std::cout << "receive a early gnss data" << std::endl;
    return false;
  }
  gnss_pose_buffer_.push_back(gnss_pose);
  if (gnss_pose_buffer_.size() > 100) {
    gnss_pose_buffer_.pop_front();
  }
  return true;
}

bool SlidingWindow::update()
{
  has_new_optimized_ = false;
  if (!has_inited_) {
    // init graph
    if (!try_init_from_lidar()) {
      return false;
    }
    has_inited_ = true;
    return true;
  }
  if (!check_valid_lidar()) {
    return false;
  }
  // get current lidar
  size_t current_lidar_idx = unhandled_lidar_idx_;
  auto & lidar_pose = lidar_pose_buffer_.at(current_lidar_idx);
  unhandled_lidar_idx_++;
  // key frame
  if (!check_new_key_frame(lidar_pose)) {
    return false;
  }
  // integrated_imu_buffer
  std::vector<localization_common::ImuData> pre_integration_buffer;
  pre_integration_buffer.push_back(current_imu_);
  get_synced_imu_buffer(lidar_pose.time, pre_integration_buffer);
  current_imu_ = pre_integration_buffer.back();
  // create graph node from lidar
  int vertex_idx = create_graph_node_from_lidar(current_lidar_idx, current_lidar_idx - 1);
  // create graph node from imu
  // int vertex_idx = create_graph_node_from_imu(pre_integration_buffer);
  // add lidar pose constraint into graph
  if (use_lidar_pose_) {
    Eigen::Matrix4d prior_pose = lidar_pose.pose * T_base_imu_;
    graph_optimizer_->add_absolute_pose_edge(vertex_idx, prior_pose, lidar_pose_noise_);
  }
  // add gnss position constraint into graph
  if (use_gnss_) {
    localization_common::OdomData synced_gnss_pose;
    get_synced_gnss(lidar_pose.time, synced_gnss_pose);
    // TODO(gezp)
  }
  // add imu pre integration constraint into graph
  if (use_imu_pre_integration_) {
    std::cout << "pre integration buffer size:" << pre_integration_buffer.size() << std::endl;
    graph_optimizer_->add_imu_pre_integration_edge(
      vertex_idx - 1, vertex_idx, pre_integration_buffer);
  }
  // update graph
  update_graph();
  latest_key_frame_ = lidar_pose;
  return true;
}

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

bool SlidingWindow::check_valid_lidar()
{
  if (lidar_pose_buffer_.size() < unhandled_lidar_idx_ + 1) {
    return false;
  }
  auto & lidar_pose = lidar_pose_buffer_.at(unhandled_lidar_idx_);
  if (imu_buffer_.back().time < lidar_pose.time) {
    // large imu delay. wait imu
    std::cout << "large imu delay" << std::endl;
    return false;
  }
  if (imu_buffer_.front().time > lidar_pose.time) {
    // drop old lidar data
    unhandled_lidar_idx_++;
    std::cout << "old lidar data" << imu_buffer_.front().time - lidar_pose.time << std::endl;
    return false;
  }
  return true;
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

int SlidingWindow::create_graph_node(
  double time, const Eigen::Matrix4d & pose, const Eigen::Vector3d & vel)
{
  localization_common::ImuNavState imu_nav_state;
  imu_nav_state.time = time;
  imu_nav_state.position = pose.block<3, 1>(0, 3);
  imu_nav_state.orientation = pose.block<3, 3>(0, 0);
  imu_nav_state.linear_velocity = vel;
  if (graph_optimizer_->get_vertex_count() > 0) {
    auto last_state = graph_optimizer_->get_imu_nav_state();
    imu_nav_state.accel_bias = last_state.accel_bias;
    imu_nav_state.gyro_bias = last_state.gyro_bias;
  }
  // add vertex
  int vertex_idx = graph_optimizer_->add_vertex(imu_nav_state, false);
  return vertex_idx;
}

int SlidingWindow::create_graph_node_from_odom(const localization_common::OdomData & odom)
{
  auto odom_imu = transform_odom(odom, T_base_imu_);
  Eigen::Vector3d vel = odom_imu.pose.block<3, 3>(0, 0) * odom_imu.linear_velocity;
  return create_graph_node(odom_imu.time, odom_imu.pose, vel);
}

int SlidingWindow::create_graph_node_from_lidar(size_t idx, size_t neighbor_idx)
{
  // get approximate average velocity between current pose and neighbor pose
  auto & lidar_pose = lidar_pose_buffer_.at(idx);
  auto & next_lidar_pose = lidar_pose_buffer_.at(neighbor_idx);
  Eigen::Matrix4d current_pose = lidar_pose.pose * T_base_imu_;
  Eigen::Matrix4d next_pose = next_lidar_pose.pose * T_base_imu_;
  Eigen::Vector3d dx = next_pose.block<3, 1>(0, 3) - current_pose.block<3, 1>(0, 3);
  Eigen::Vector3d current_vel = dx / (next_lidar_pose.time - lidar_pose.time);
  return create_graph_node(lidar_pose.time, current_pose, current_vel);
}

int SlidingWindow::create_graph_node_from_imu(
  const std::vector<localization_common::ImuData> & imus)
{
  auto last_state = graph_optimizer_->get_imu_nav_state();
  imu_odometry::ImuIntegration integration;
  integration.reset(last_state);
  for (auto & imu : imus) {
    integration.integrate(imu);
  }
  // add vertex
  int vertex_idx = graph_optimizer_->add_vertex(integration.get_imu_nav_state(), false);
  return vertex_idx;
}

bool SlidingWindow::get_synced_imu_buffer(
  double time, std::vector<localization_common::ImuData> & buffer)
{
  if (imu_buffer_.empty() || imu_buffer_.back().time < time) {
    return false;
  }
  localization_common::ImuData imu;
  while (imu_buffer_.front().time <= time) {
    imu = imu_buffer_.front();
    imu_buffer_.pop_front();
    buffer.push_back(imu);
  }
  // get sync imu
  if (imu.time != time) {
    imu = localization_common::interpolate_imu(imu, imu_buffer_.front(), time);
    buffer.push_back(imu);
  }
  return true;
}

bool SlidingWindow::get_synced_gnss(double time, localization_common::OdomData & odom)
{
  if (gnss_pose_buffer_.empty() || gnss_pose_buffer_.back().time < time) {
    return false;
  }
  if (gnss_pose_buffer_.front().time > time) {
    return false;
  }
  if (gnss_pose_buffer_.size() == 1) {
    odom = gnss_pose_buffer_.at(1);
    return true;
  }
  while (gnss_pose_buffer_.at(1).time < time) {
    gnss_pose_buffer_.pop_front();
  }
  if (gnss_pose_buffer_.at(1).time == time) {
    odom = gnss_pose_buffer_.at(1);
    gnss_pose_buffer_.pop_front();
  } else {
    // TODO(gezp): interpolate odom(0, 1)
    std::cout << "unsynced gnss" << std::endl;
    return false;
  }
  return true;
}

bool SlidingWindow::try_init_from_lidar()
{
  if (lidar_pose_buffer_.size() < unhandled_lidar_idx_ + 2) {
    return false;
  }
  if (!check_valid_lidar()) {
    return false;
  }
  // get current lidar
  size_t current_lidar_idx = unhandled_lidar_idx_;
  auto lidar_pose = lidar_pose_buffer_.at(current_lidar_idx);
  unhandled_lidar_idx_++;
  // get current imu
  std::vector<localization_common::ImuData> synced_imu_buffer;
  get_synced_imu_buffer(lidar_pose.time, synced_imu_buffer);
  current_imu_ = synced_imu_buffer.back();
  // create graph node from lidar
  int vertex_idx = create_graph_node_from_lidar(current_lidar_idx, current_lidar_idx + 1);
  // add lidar pose constraint into graph
  if (use_lidar_pose_) {
    Eigen::Matrix4d prior_pose = lidar_pose.pose * T_base_imu_;
    graph_optimizer_->add_absolute_pose_edge(vertex_idx, prior_pose, lidar_pose_noise_);
  }
  latest_key_frame_ = lidar_pose;
  std::cout << "try_init_from_lidar successfully!" << std::endl;
  return true;
}

bool SlidingWindow::update_graph()
{
  if (graph_optimizer_->optimize()) {
    if (graph_optimizer_->get_vertex_count() > sliding_window_size_) {
      graph_optimizer_->marginalize(1);
    }
    has_new_optimized_ = true;
  }
  return true;
}

}  // namespace graph_based_localization
