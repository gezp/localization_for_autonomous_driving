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

#include <filesystem>

#include "localization_common/sensor_data_utils.hpp"

namespace loosely_lio_mapping
{

LioBackEnd::LioBackEnd()
{
  cloud_filter_factory_ = std::make_shared<localization_common::CloudFilterFactory>();
}

bool LioBackEnd::init_config(const std::string & config_path, const std::string & data_path)
{
  YAML::Node config_node = YAML::LoadFile(config_path);
  // init_param
  key_frame_distance_ = config_node["key_frame_distance"].as<float>();
  gravity_magnitude_ = config_node["earth"]["gravity_magnitude"].as<double>();
  use_gnss_ = config_node["use_gnss"].as<bool>();
  use_loop_closure_ = config_node["use_loop_closure"].as<bool>();
  use_imu_pre_integration_ = config_node["use_imu_pre_integration"].as<bool>();
  optimize_step_with_key_frame = (config_node["optimization_step_size"]["key_frame"].as<int>());
  optimize_step_with_loop = (config_node["optimization_step_size"]["loop_closure"].as<int>());
  // init graph optimizer
  init_graph_optimizer(config_node);
  // init key_frame_manager
  key_frame_manager_ = std::make_shared<localization_common::LidarKeyFrameManager>(data_path);
  // init filter
  display_filter_ = cloud_filter_factory_->create(config_node["display_filter"]);
  global_map_filter_ = cloud_filter_factory_->create(config_node["global_map_filter"]);
  // print info
  std::cout << "display filter:" << std::endl;
  display_filter_->print_info();
  std::cout << "global_map filter:" << std::endl;
  global_map_filter_->print_info();
  return true;
}

void LioBackEnd::set_imu_extrinsic(const Eigen::Matrix4f & T_base_imu)
{
  T_base_imu_ = T_base_imu.cast<double>();
}

bool LioBackEnd::update(
  const localization_common::LidarData<pcl::PointXYZ> & lidar_data,
  const localization_common::OdomData & lidar_odom, const localization_common::OdomData & gnss_odom,
  const localization_common::ImuData & imu_data)
{
  has_new_key_frame_ = false;
  has_new_optimized_ = false;
  if (use_imu_pre_integration_) {
    imu_buffer_.push_back(imu_data);
  }
  if (check_new_key_frame(lidar_odom)) {
    has_new_key_frame_ = true;
    // add new key_frame
    Eigen::Matrix4d pose = pose_to_optimize_ * lidar_odom.pose;
    key_frame_manager_->add_key_frame(lidar_odom.time, pose, lidar_data.point_cloud);
    new_key_frame_cnt_++;
    //
    current_lidar_pose_ = lidar_odom.pose;
    current_gnss_pose_ = gnss_odom.pose;
    current_gnss_twist_.linear_velocity = gnss_odom.linear_velocity.cast<float>();
    current_gnss_twist_.angular_velocity = gnss_odom.angular_velocity.cast<float>();
    // add node
    add_node_and_edge();
    if (optimize(false)) {
      has_new_optimized_ = true;
    }
    last_lidar_pose_ = current_lidar_pose_;
    return true;
  }
  return false;
}

bool LioBackEnd::insert_loop_candidate(const localization_common::LoopCandidate & loop_candidate)
{
  if (!use_loop_closure_) {
    return false;
  }
  Eigen::Matrix4d relative_pose = T_base_imu_.inverse() * loop_candidate.pose * T_base_imu_;
  // add constraint loop closure detection:
  graph_optimizer_->add_relative_pose_edge(
    loop_candidate.index1, loop_candidate.index2, relative_pose, loop_closure_noise_);
  new_loop_cnt_++;
  return true;
}

bool LioBackEnd::add_raw_imu(const localization_common::ImuData & imu_data)
{
  if (!use_imu_pre_integration_) {
    return false;
  }
  if (key_frame_manager_->get_key_frame_count() == 0) {
    return false;
  }
  if (imu_data.time <= key_frame_manager_->get_key_frames().back().time) {
    return false;
  }
  imu_buffer_.push_back(imu_data);
  return true;
}

bool LioBackEnd::optimize(bool force)
{
  if (
    (!force) && (new_key_frame_cnt_ < optimize_step_with_key_frame) &&
    (new_loop_cnt_ < optimize_step_with_loop))
  {
    return false;
  }
  // optimize
  if (!graph_optimizer_->optimize()) {
    return false;
  }
  // reset key frame counters:
  new_key_frame_cnt_ = new_loop_cnt_ = 0;
  // update key frames
  auto optimized_states = graph_optimizer_->get_optimized_vertices();
  assert(optimized_states.size() == key_frame_manager_->get_key_frame_count());
  for (size_t i = 0; i < optimized_states.size(); ++i) {
    Eigen::Matrix4d imu_pose = Eigen::Matrix4d::Identity();
    imu_pose.block<3, 1>(0, 3) = optimized_states[i].position;
    imu_pose.block<3, 3>(0, 0) = optimized_states[i].orientation;
    Eigen::Matrix4d pose = imu_pose * T_base_imu_.inverse();
    key_frame_manager_->update_key_frame(i, pose);
  }
  // update pose_to_optimize_
  pose_to_optimize_ =
    key_frame_manager_->get_key_frames().back().pose * current_lidar_pose_.inverse();
  return true;
}

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

bool LioBackEnd::add_node_and_edge()
{
  // add node for new key frame pose:
  // fix the pose of the first key frame for lidar only mapping:
  localization_common::ImuNavState imu_nav_state;
  auto keyframe = key_frame_manager_->get_key_frames().back();
  if (!use_gnss_ && graph_optimizer_->get_vertex_num() == 0) {
    imu_nav_state.time = keyframe.time;
    Eigen::Matrix4d pose = current_lidar_pose_ * T_base_imu_;
    imu_nav_state.position = pose.block<3, 1>(0, 3);
    imu_nav_state.orientation = pose.block<3, 3>(0, 0);
    graph_optimizer_->add_vertex(imu_nav_state, true);
  } else {
    imu_nav_state.time = keyframe.time;
    Eigen::Matrix4d pose = current_gnss_pose_ * T_base_imu_;
    imu_nav_state.position = pose.block<3, 1>(0, 3);
    imu_nav_state.orientation = pose.block<3, 3>(0, 0);
    auto vel2 = transform_velocity_data(current_gnss_twist_, T_base_imu_.cast<float>());
    imu_nav_state.linear_velocity = imu_nav_state.orientation * vel2.linear_velocity.cast<double>();
    graph_optimizer_->add_vertex(imu_nav_state, false);
  }
  // add constraints:
  // lidar frontend
  int n = graph_optimizer_->get_vertex_num();
  if (n > 1) {
    Eigen::Matrix4d last_pose = last_lidar_pose_ * T_base_imu_;
    Eigen::Matrix4d cur_pose = current_lidar_pose_ * T_base_imu_;
    Eigen::Matrix4d relative_pose = last_pose.inverse() * cur_pose;
    graph_optimizer_->add_relative_pose_edge(n - 2, n - 1, relative_pose, lidar_odom_noise_);
  }
  // gnss position
  if (use_gnss_) {
    // get prior position measurement:
    Eigen::Matrix4d pose = current_gnss_pose_ * T_base_imu_;
    Eigen::Vector3d pos = pose.block<3, 1>(0, 3);
    // add constraint, GNSS position:
    graph_optimizer_->add_prior_position_edge(n - 1, pos, gnss_noise_);
  }

  // IMU pre-integration:
  if (use_imu_pre_integration_ && n > 1) {
    // add constraint, IMU pre-integraion:
    graph_optimizer_->add_imu_pre_integration_edge(n - 2, n - 1, imu_buffer_);
    imu_buffer_.clear();
  }
  return true;
}

bool LioBackEnd::check_new_key_frame(const localization_common::OdomData & lidar_odom)
{
  if (key_frame_manager_->get_key_frame_count() == 0) {
    return true;
  }
  Eigen::Vector3d translation =
    (lidar_odom.pose.block<3, 1>(0, 3) - last_lidar_pose_.block<3, 1>(0, 3));
  // whether the current scan is far away enough from last key frame:
  if (translation.lpNorm<1>() > key_frame_distance_) {
    return true;
  }
  return false;
}

bool LioBackEnd::has_new_key_frame() {return has_new_key_frame_;}

bool LioBackEnd::has_new_optimized() {return has_new_optimized_;}

const std::vector<localization_common::LidarFrame> & LioBackEnd::get_key_frames()
{
  return key_frame_manager_->get_key_frames();
}

Eigen::Matrix4d LioBackEnd::get_lidar_odom_to_map() {return pose_to_optimize_;}

pcl::PointCloud<pcl::PointXYZ>::Ptr LioBackEnd::get_global_map()
{
  return key_frame_manager_->get_global_map(display_filter_);
}

bool LioBackEnd::save_map()
{
  key_frame_manager_->save_key_frame_pose();
  key_frame_manager_->save_global_map(global_map_filter_);
  return true;
}

}  // namespace loosely_lio_mapping
