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

#include "loosely_lio_mapping/lio_back_end.hpp"

#include <pcl/io/pcd_io.h>

#include <Eigen/Dense>
#include <filesystem>
#include <fstream>

#include "localization_common/sensor_data_utils.hpp"

namespace loosely_lio_mapping
{

LioBackEnd::LioBackEnd() {}

bool LioBackEnd::init_config(const std::string & config_path, const std::string & data_path)
{
  YAML::Node config_node = YAML::LoadFile(config_path);
  // init_param
  key_frame_distance_ = config_node["key_frame_distance"].as<float>();
  // init_data_path
  data_path_ = data_path;
  key_frames_path_ = data_path_ + "/key_frames";
  if (std::filesystem::is_directory(key_frames_path_)) {
    std::filesystem::remove_all(key_frames_path_);
  }
  if (!std::filesystem::create_directory(key_frames_path_)) {
    return false;
  }
  // params
  gravity_magnitude_ = config_node["earth"]["gravity_magnitude"].as<double>();
  use_gnss_ = config_node["use_gnss"].as<bool>();
  use_loop_closure_ = config_node["use_loop_closure"].as<bool>();
  use_imu_pre_integration_ = config_node["use_imu_pre_integration"].as<bool>();
  optimization_step_size_.key_frame =
    (config_node["optimization_step_size"]["key_frame"].as<int>());
  optimization_step_size_.loop_closure =
    (config_node["optimization_step_size"]["loop_closure"].as<int>());
  // init graph optimizer
  init_graph_optimizer(config_node);
  return true;
}

void LioBackEnd::set_imu_extrinsic(const Eigen::Matrix4f & T_base_imu) {T_base_imu_ = T_base_imu;}

bool LioBackEnd::update(
  const localization_common::LidarData<pcl::PointXYZ> & lidar_data,
  const localization_common::OdomData & lidar_odom, const localization_common::OdomData & gnss_pose,
  const localization_common::ImuData & imu_data)
{
  has_new_key_frame_ = false;
  has_new_optimized_ = false;
  if (use_imu_pre_integration_) {
    imu_buffer_.push_back(imu_data);
  }
  if (check_new_key_frame(lidar_odom)) {
    has_new_key_frame_ = true;
    add_new_key_frame(lidar_data, lidar_odom, gnss_pose);
    add_node_and_edge();
    if (check_need_optimize()) {
      optimize();
    } else {
      localization_common::KeyFrame optimized_key_frame = current_key_frame_;
      optimized_key_frame.pose = pose_to_optimize_ * optimized_key_frame.pose;
      optimized_key_frames_.push_back(std::move(optimized_key_frame));
    }
    return true;
  }
  return false;
}

bool LioBackEnd::insert_loop_pose(const localization_common::LoopPose & loop_pose)
{
  if (!use_loop_closure_) {
    return false;
  }
  Eigen::Matrix4f relative_pose = T_base_imu_.inverse() * loop_pose.pose * T_base_imu_;
  // add constraint, lidar frontend / loop closure detection:
  graph_optimizer_->add_relative_pose_edge(
    loop_pose.index0, loop_pose.index1, relative_pose.cast<double>(), loop_closure_noise_);
  new_loop_cnt_++;
  std::cout << "Add loop closure: " << loop_pose.index0 << "," << loop_pose.index1 << std::endl;
  return true;
}

bool LioBackEnd::add_raw_imu(const localization_common::ImuData & imu_data)
{
  if (!use_imu_pre_integration_) {
    return false;
  }
  if (key_frames_.size() == 0) {
    return false;
  }
  if (imu_data.time <= key_frames_.back().time) {
    return false;
  }
  imu_buffer_.push_back(imu_data);
  return true;
}

bool LioBackEnd::optimize()
{
  std::cout << "optimize" << std::endl;
  if (graph_optimizer_->optimize()) {
    has_new_optimized_ = true;
  }
  // update optimized key frames
  optimized_key_frames_.clear();
  auto optimized_states = graph_optimizer_->get_optimized_vertices();
  for (size_t i = 0; i < optimized_states.size(); ++i) {
    localization_common::KeyFrame key_frame;
    key_frame.index = i;
    key_frame.time = optimized_states[i].time;
    Eigen::Matrix4d imu_pose = Eigen::Matrix4d::Identity();
    imu_pose.block<3, 1>(0, 3) = optimized_states[i].position;
    imu_pose.block<3, 3>(0, 0) = optimized_states[i].orientation;
    key_frame.pose = imu_pose.cast<float>() * T_base_imu_.inverse();
    optimized_key_frames_.push_back(std::move(key_frame));
  }
  // update pose_to_optimize_
  assert(optimized_key_frames_.size() == key_frames_.size());
  pose_to_optimize_ = optimized_key_frames_.back().pose * key_frames_.back().pose.inverse();
  return has_new_optimized_;
}

bool LioBackEnd::has_new_key_frame() {return has_new_key_frame_;}

bool LioBackEnd::has_new_optimized() {return has_new_optimized_;}

void LioBackEnd::get_latest_key_scan(localization_common::LidarData<pcl::PointXYZ> & key_scan)
{
  key_scan.time = current_key_scan_.time;
  key_scan.point_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>(*current_key_scan_.point_cloud));
}

void LioBackEnd::get_latest_key_frame(localization_common::KeyFrame & key_frame)
{
  key_frame = current_key_frame_;
}

void LioBackEnd::get_latest_key_gnss(localization_common::KeyFrame & key_frame)
{
  key_frame = current_key_gnss_;
}

std::deque<localization_common::KeyFrame> LioBackEnd::get_optimized_key_frames()
{
  return optimized_key_frames_;
}

Eigen::Matrix4f LioBackEnd::get_odom_to_map() {return pose_to_optimize_;}

bool LioBackEnd::init_graph_optimizer(const YAML::Node & config_node)
{
  std::string graph_optimizer_method = config_node["graph_optimizer_method"].as<std::string>();
  if (graph_optimizer_method == "g2o") {
    auto g2o_optimizer = std::make_shared<G2oGraphOptimizer>(config_node[graph_optimizer_method]);
    Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -gravity_magnitude_);
    g2o_optimizer->set_gravity(gravity);
    graph_optimizer_ = g2o_optimizer;
  } else {
    std::cerr << "Optimizer " << graph_optimizer_method << " NOT FOUND!";
    return false;
  }
  std::cout << "\tOptimizer:" << graph_optimizer_method << std::endl << std::endl;
  lidar_odom_noise_.resize(6);
  loop_closure_noise_.resize(6);
  gnss_noise_.resize(3);
  // x-y-z & yaw-roll-pitch
  for (int i = 0; i < 6; ++i) {
    lidar_odom_noise_(i) = config_node[graph_optimizer_method]["lidar_odom_noise"][i].as<double>();
    loop_closure_noise_(i) =
      config_node[graph_optimizer_method]["loop_closure_noise"][i].as<double>();
  }
  // x-y-z:
  for (int i = 0; i < 3; i++) {
    gnss_noise_(i) = config_node[graph_optimizer_method]["gnss_noise"][i].as<double>();
  }
  return true;
}

bool LioBackEnd::add_new_key_frame(
  const localization_common::LidarData<pcl::PointXYZ> & lidar_data,
  const localization_common::OdomData & lidar_odom, const localization_common::OdomData & gnss_odom)
{
  // write new key scan to disk
  std::string file_path =
    key_frames_path_ + "/key_frame_" + std::to_string(key_frames_.size()) + ".pcd";
  pcl::io::savePCDFileBinary(file_path, *lidar_data.point_cloud);
  current_key_scan_.time = lidar_data.time;
  current_key_scan_.point_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>(*lidar_data.point_cloud));
  // key frame
  localization_common::KeyFrame key_frame;
  key_frame.time = lidar_odom.time;
  key_frame.index = (unsigned int)key_frames_.size();
  key_frame.pose = lidar_odom.pose.cast<float>();
  key_frames_.push_back(key_frame);
  current_key_frame_ = key_frame;
  // key gnss
  current_key_gnss_.time = current_key_frame_.time;
  current_key_gnss_.index = current_key_frame_.index;
  current_key_gnss_.pose = gnss_odom.pose.cast<float>();
  current_key_gnss_.vel.v = gnss_odom.linear_velocity.cast<float>();
  current_key_gnss_.vel.w = gnss_odom.angular_velocity.cast<float>();
  return true;
}

bool LioBackEnd::add_node_and_edge()
{
  static localization_common::KeyFrame last_key_frame_ = current_key_frame_;
  // add node for new key frame pose:
  // fix the pose of the first key frame for lidar only mapping:
  localization_common::ImuNavState imu_nav_state;
  if (!use_gnss_ && graph_optimizer_->get_vertex_num() == 0) {
    imu_nav_state.time = current_key_frame_.time;
    Eigen::Matrix4f pose = current_key_frame_.pose * T_base_imu_;
    imu_nav_state.position = pose.block<3, 1>(0, 3).cast<double>();
    imu_nav_state.orientation = pose.block<3, 3>(0, 0).cast<double>();
    graph_optimizer_->add_vertex(imu_nav_state, true);
  } else {
    imu_nav_state.time = current_key_gnss_.time;
    Eigen::Matrix4f pose = current_key_gnss_.pose * T_base_imu_;
    imu_nav_state.position = pose.block<3, 1>(0, 3).cast<double>();
    imu_nav_state.orientation = pose.block<3, 3>(0, 0).cast<double>();
    localization_common::VelocityData vel;
    vel.linear_velocity = current_key_gnss_.vel.v;
    vel.angular_velocity = current_key_gnss_.vel.w;
    auto vel2 = transform_velocity_data(vel, T_base_imu_);
    imu_nav_state.linear_velocity = imu_nav_state.orientation * vel2.linear_velocity.cast<double>();
    graph_optimizer_->add_vertex(imu_nav_state, false);
  }
  // add constraints:
  // a. lidar frontend / loop closure detection:
  int n = graph_optimizer_->get_vertex_num();
  if (n > 1) {
    Eigen::Matrix4f last_pose = last_key_frame_.pose * T_base_imu_;
    Eigen::Matrix4f cur_pose = current_key_frame_.pose * T_base_imu_;
    Eigen::Matrix4d relative_pose = (last_pose.inverse() * cur_pose).cast<double>();
    graph_optimizer_->add_relative_pose_edge(n - 2, n - 1, relative_pose, lidar_odom_noise_);
  }

  // b. GNSS position:
  if (use_gnss_) {
    // get prior position measurement:
    Eigen::Matrix4f pose = current_key_gnss_.pose * T_base_imu_;
    Eigen::Vector3d pos = pose.block<3, 1>(0, 3).cast<double>();
    // add constraint, GNSS position:
    graph_optimizer_->add_prior_position_edge(n - 1, pos, gnss_noise_);
    new_gnss_cnt_++;
  }

  // c. IMU pre-integration:
  if (n > 1 && use_imu_pre_integration_) {
    // add constraint, IMU pre-integraion:
    graph_optimizer_->add_imu_pre_integration_edge(n - 2, n - 1, imu_buffer_);
    imu_buffer_.clear();
  }

  // move forward:
  last_key_frame_ = current_key_frame_;
  new_key_frame_cnt_++;
  return true;
}

bool LioBackEnd::check_new_key_frame(const localization_common::OdomData & lidar_odom)
{
  if (key_frames_.size() == 0) {
    return true;
  }
  auto last_key_pose = key_frames_.back().pose;
  // whether the current scan is far away enough from last key frame:
  if (
    fabs(lidar_odom.pose(0, 3) - last_key_pose(0, 3)) +
    fabs(lidar_odom.pose(1, 3) - last_key_pose(1, 3)) +
    fabs(lidar_odom.pose(2, 3) - last_key_pose(2, 3)) >
    key_frame_distance_)
  {
    return true;
  }
  return false;
}

bool LioBackEnd::check_need_optimize()
{
  bool need_optimize = false;

  if (new_key_frame_cnt_ >= optimization_step_size_.key_frame) {
    new_key_frame_cnt_ = 0;
    need_optimize = true;
  }
  if (new_loop_cnt_ >= optimization_step_size_.loop_closure) {
    new_loop_cnt_ = 0;
    need_optimize = true;
  }
  return need_optimize;
}

}  // namespace loosely_lio_mapping
