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

#include "lidar_mapping/back_end.hpp"

#include <pcl/io/pcd_io.h>

#include <Eigen/Dense>
#include <filesystem>
#include <fstream>

namespace lidar_mapping
{
BackEnd::BackEnd() {}

bool BackEnd::init_config(const std::string & config_path, const std::string & data_path)
{
  YAML::Node config_node = YAML::LoadFile(config_path);
  // init_param
  key_frame_distance_ = config_node["key_frame_distance"].as<float>();
  // init graph optimizer
  init_graph_optimizer(config_node);
  // init_data_path
  data_path_ = data_path;
  key_frames_path_ = data_path_ + "/key_frames";
  if (std::filesystem::is_directory(key_frames_path_)) {
    std::filesystem::remove_all(key_frames_path_);
  }
  if (!std::filesystem::create_directory(key_frames_path_)) {
    return false;
  }
  // init map_generator
  map_generator_ = std::make_shared<MapGenerator>();
  map_generator_->init_config(config_path, data_path);
  return true;
}

bool BackEnd::init_graph_optimizer(const YAML::Node & config_node)
{
  std::string graph_optimizer_method = config_node["graph_optimizer_method"].as<std::string>();
  if (graph_optimizer_method == "g2o") {
    graph_optimizer_ = std::make_shared<G2oGraphOptimizer>("lm_var_csparse");
  } else {
    std::cerr << "Optimizer " << graph_optimizer_method << " NOT FOUND!";
    return false;
  }
  std::cout << "\tOptimizer:" << graph_optimizer_method << std::endl << std::endl;

  graph_optimizer_config_.use_gnss = config_node["use_gnss"].as<bool>();
  graph_optimizer_config_.use_loop_close = config_node["use_loop_close"].as<bool>();

  graph_optimizer_config_.optimize_step_with_key_frame =
    config_node["optimize_step_with_key_frame"].as<int>();
  graph_optimizer_config_.optimize_step_with_gnss =
    config_node["optimize_step_with_gnss"].as<int>();
  graph_optimizer_config_.optimize_step_with_loop =
    config_node["optimize_step_with_loop"].as<int>();

  // x-y-z & yaw-roll-pitch
  for (int i = 0; i < 6; ++i) {
    graph_optimizer_config_.odom_edge_noise(i) =
      config_node[graph_optimizer_method]["odom_edge_noise"][i].as<double>();
    graph_optimizer_config_.close_loop_noise(i) =
      config_node[graph_optimizer_method]["close_loop_noise"][i].as<double>();
  }

  // x-y-z:
  for (int i = 0; i < 3; i++) {
    graph_optimizer_config_.gnss_noise(i) =
      config_node[graph_optimizer_method]["gnss_noise"][i].as<double>();
  }

  return true;
}

bool BackEnd::update(
  const localization_common::LidarData<pcl::PointXYZ> & lidar_data,
  const localization_common::OdomData & lidar_odom, const localization_common::OdomData & gnss_pose)
{
  has_new_key_frame_ = false;
  has_new_optimized_ = false;
  if (check_new_key_frame(lidar_odom)) {
    has_new_key_frame_ = true;
    add_new_key_frame(lidar_data, lidar_odom, gnss_pose);
    add_node_and_edge(gnss_pose);
    if (check_need_optimize()) {
      optimize();
    } else {
      localization_common::KeyFrame optimized_key_frame = current_key_frame_;
      optimized_key_frame.pose = pose_to_optimize_ * optimized_key_frame.pose;
      optimized_key_frames_.push_back(std::move(optimized_key_frame));
    }
    last_key_frame_ = current_key_frame_;
    return true;
  }
  return false;
}

bool BackEnd::insert_loop_pose(const localization_common::LoopPose & loop_pose)
{
  if (!graph_optimizer_config_.use_loop_close) {
    return false;
  }

  Eigen::Isometry3d isometry;
  isometry.matrix() = loop_pose.pose.cast<double>();
  graph_optimizer_->add_relative_pose_edge(
    loop_pose.index0, loop_pose.index1, isometry, graph_optimizer_config_.close_loop_noise);

  new_loop_cnt_++;

  std::cout << "Add loop closure: " << loop_pose.index0 << "," << loop_pose.index1 << std::endl;

  return true;
}

bool BackEnd::check_new_key_frame(const localization_common::OdomData & lidar_odom)
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

bool BackEnd::add_new_key_frame(
  const localization_common::LidarData<pcl::PointXYZ> & lidar_data,
  const localization_common::OdomData & lidar_odom, const localization_common::OdomData & gnss_odom)
{
  // a. first write new key scan to disk:
  std::string file_path =
    key_frames_path_ + "/key_frame_" + std::to_string(key_frames_.size()) + ".pcd";
  pcl::io::savePCDFileBinary(file_path, *lidar_data.point_cloud);

  // b. create key frame index for lidar scan:
  localization_common::KeyFrame key_frame;
  key_frame.time = lidar_odom.time;
  key_frame.index = (unsigned int)key_frames_.size();
  key_frame.pose = lidar_odom.pose.cast<float>();
  key_frames_.push_back(key_frame);
  current_key_frame_ = key_frame;
  //
  current_gnss_pose_ = gnss_odom.pose;

  return true;
}

bool BackEnd::add_node_and_edge(const localization_common::OdomData & gnss_data)
{
  // add node for new key frame pose:
  Eigen::Isometry3d isometry;
  isometry.matrix() = current_key_frame_.pose.cast<double>();
  // fix the pose of the first key frame:
  if (!graph_optimizer_config_.use_gnss && graph_optimizer_->get_node_num() == 0) {
    graph_optimizer_->add_node(isometry, true);
  } else {
    graph_optimizer_->add_node(isometry, false);
  }
  new_key_frame_cnt_++;

  // add edge for new key frame:
  int node_num = graph_optimizer_->get_node_num();
  if (node_num > 1) {
    Eigen::Matrix4f relative_pose = last_key_frame_.pose.inverse() * current_key_frame_.pose;
    isometry.matrix() = relative_pose.cast<double>();
    graph_optimizer_->add_relative_pose_edge(
      node_num - 2, node_num - 1, isometry, graph_optimizer_config_.odom_edge_noise);
  }

  // add prior for new key frame pose using GNSS/IMU estimation:
  if (graph_optimizer_config_.use_gnss) {
    Eigen::Vector3d xyz = gnss_data.pose.block<3, 1>(0, 3);
    graph_optimizer_->add_prior_xyz_edge(node_num - 1, xyz, graph_optimizer_config_.gnss_noise);
    new_gnss_cnt_++;
  }

  return true;
}

bool BackEnd::check_need_optimize()
{
  bool need_optimize = false;
  if (
    new_key_frame_cnt_ >= graph_optimizer_config_.optimize_step_with_key_frame ||
    new_gnss_cnt_ >= graph_optimizer_config_.optimize_step_with_gnss ||
    new_loop_cnt_ >= graph_optimizer_config_.optimize_step_with_loop)
  {
    need_optimize = true;
  }
  return need_optimize;
}

bool BackEnd::optimize()
{
  // reset key frame counters:
  new_key_frame_cnt_ = new_gnss_cnt_ = new_loop_cnt_ = 0;
  if (graph_optimizer_->optimize()) {
    has_new_optimized_ = true;
  }
  // update optimized key frames
  optimized_key_frames_.clear();
  std::deque<Eigen::Matrix4f> optimized_pose;
  graph_optimizer_->get_optimized_pose(optimized_pose);
  for (size_t i = 0; i < optimized_pose.size(); ++i) {
    localization_common::KeyFrame key_frame;
    key_frame.index = key_frames_.at(i).index;
    key_frame.time = key_frames_.at(i).time;
    key_frame.pose = optimized_pose.at(i);
    optimized_key_frames_.push_back(std::move(key_frame));
  }
  // update pose_to_optimize_
  assert(optimized_key_frames_.size() == key_frames_.size());
  pose_to_optimize_ = optimized_key_frames_.back().pose * key_frames_.back().pose.inverse();
  return has_new_optimized_;
}

bool BackEnd::has_new_key_frame() {return has_new_key_frame_;}

bool BackEnd::has_new_optimized() {return has_new_optimized_;}

void BackEnd::get_latest_key_frame(localization_common::KeyFrame & key_frame)
{
  key_frame = current_key_frame_;
  key_frame.pose = current_gnss_pose_.cast<float>();
}

std::deque<localization_common::KeyFrame> BackEnd::get_optimized_key_frames()
{
  return optimized_key_frames_;
}

Eigen::Matrix4f BackEnd::get_lidar_odom_to_map() {return pose_to_optimize_;}

pcl::PointCloud<pcl::PointXYZ>::Ptr BackEnd::get_global_map(bool use_display_filter)
{
  return map_generator_->get_global_map(optimized_key_frames_, use_display_filter);
}
bool BackEnd::save_map() {return map_generator_->save_map(optimized_key_frames_);}
}  // namespace lidar_mapping
