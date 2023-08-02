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

namespace lidar_mapping
{
BackEnd::BackEnd()
{
  cloud_filter_factory_ = std::make_shared<localization_common::CloudFilterFactory>();
}

bool BackEnd::init_config(const std::string & config_path, const std::string & data_path)
{
  YAML::Node config_node = YAML::LoadFile(config_path);
  // init_param
  key_frame_distance_ = config_node["key_frame_distance"].as<float>();
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
  const localization_common::OdomData & lidar_odom, const localization_common::OdomData & gnss_odom)
{
  has_new_key_frame_ = false;
  has_new_optimized_ = false;
  if (check_new_key_frame(lidar_odom)) {
    has_new_key_frame_ = true;
    // add new key_frame
    Eigen::Matrix4d pose = pose_to_optimize_ * lidar_odom.pose;
    key_frame_manager_->add_key_frame(lidar_odom.time, pose, lidar_data.point_cloud);
    //
    current_lidar_pose_ = lidar_odom.pose;
    current_gnss_pose_ = gnss_odom.pose;
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
  return true;
}

bool BackEnd::check_new_key_frame(const localization_common::OdomData & lidar_odom)
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

bool BackEnd::add_node_and_edge()
{
  // add node for new key frame pose:
  Eigen::Isometry3d isometry;
  isometry.matrix() = key_frame_manager_->get_key_frames().back().pose;
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
    Eigen::Matrix4d relative_pose = last_lidar_pose_.inverse() * current_lidar_pose_;
    isometry.matrix() = relative_pose.cast<double>();
    graph_optimizer_->add_relative_pose_edge(
      node_num - 2, node_num - 1, isometry, graph_optimizer_config_.odom_edge_noise);
  }
  // add prior for new key frame pose using GNSS/IMU estimation:
  if (graph_optimizer_config_.use_gnss) {
    Eigen::Vector3d xyz = current_gnss_pose_.block<3, 1>(0, 3);
    graph_optimizer_->add_prior_xyz_edge(node_num - 1, xyz, graph_optimizer_config_.gnss_noise);
    new_gnss_cnt_++;
  }
  return true;
}

bool BackEnd::optimize(bool force)
{
  // check
  if (
    (!force) && (new_key_frame_cnt_ < graph_optimizer_config_.optimize_step_with_key_frame) &&
    (new_gnss_cnt_ < graph_optimizer_config_.optimize_step_with_gnss) &&
    (new_loop_cnt_ < graph_optimizer_config_.optimize_step_with_loop))
  {
    return false;
  }
  // optimize
  if (!graph_optimizer_->optimize()) {
    return false;
  }
  // reset key frame counters:
  new_key_frame_cnt_ = new_gnss_cnt_ = new_loop_cnt_ = 0;
  // update key frames
  std::deque<Eigen::Matrix4f> optimized_pose;
  graph_optimizer_->get_optimized_pose(optimized_pose);
  assert(optimized_pose.size() == key_frame_manager_->get_key_frame_count());
  for (size_t i = 0; i < optimized_pose.size(); ++i) {
    key_frame_manager_->update_key_frame(i, optimized_pose[i].cast<double>());
  }
  // update pose_to_optimize_
  pose_to_optimize_ = optimized_pose.back().cast<double>() * current_lidar_pose_.inverse();
  return true;
}

bool BackEnd::has_new_key_frame() {return has_new_key_frame_;}

bool BackEnd::has_new_optimized() {return has_new_optimized_;}

const std::vector<localization_common::LidarFrame> & BackEnd::get_key_frames()
{
  return key_frame_manager_->get_key_frames();
}

Eigen::Matrix4d BackEnd::get_lidar_odom_to_map() {return pose_to_optimize_;}

pcl::PointCloud<pcl::PointXYZ>::Ptr BackEnd::get_global_map()
{
  return key_frame_manager_->get_global_map(display_filter_);
}

bool BackEnd::save_map()
{
  key_frame_manager_->save_key_frame_pose();
  key_frame_manager_->save_global_map(global_map_filter_);
  return true;
}
}  // namespace lidar_mapping
