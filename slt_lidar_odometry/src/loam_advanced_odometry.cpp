// Copyright 2026 Gezp (https://github.com/gezp).
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

#include "slt_lidar_odometry/loam_advanced_odometry.hpp"

#include "slt_common/sensor_data_utils.hpp"
#include "slt_common/lidar_utils.hpp"

namespace slt_lidar_odometry
{

LoamAdvancedOdometry::LoamAdvancedOdometry(const YAML::Node & config)
{
  // init feature extraction, registration and filters
  feature_extraction_ =
    std::make_shared<LoamAdvancedFeatureExtraction>(config["feature_extraction"]);
  registration_ = std::make_shared<LoamAdvancedRegistration>(config["registration"]);
  // registration input filters
  edge_input_filter_ = std::make_shared<slt_common::VoxelFilter>(config["edge_input_filter"]);
  surf_input_filter_ = std::make_shared<slt_common::VoxelFilter>(config["surf_input_filter"]);
  // local map
  key_frame_distance_ = config["local_map"]["key_frame_distance"].as<double>();
  key_frame_angle_ = config["local_map"]["key_frame_angle"].as<double>();
  local_frame_num_ = config["local_map"]["local_frame_num"].as<int>();
  edge_map_filter_ = std::make_shared<slt_common::VoxelFilter>(
    config["local_map"]["edge_map_filter"]);
  surf_map_filter_ = std::make_shared<slt_common::VoxelFilter>(
    config["local_map"]["surf_map_filter"]);
  // display
  display_filter_ = std::make_shared<slt_common::VoxelFilter>(config["display_filter"]);
  edge_rgb_ = config["feature_color_rgb"]["edge"].as<std::vector<int>>();
  assert(edge_rgb_.size() == 3);
  surf_rgb_ = config["feature_color_rgb"]["surf"].as<std::vector<int>>();
  assert(surf_rgb_.size() == 3);
  bool enabel = config["enable_elapsed_time_statistics"].as<bool>();
  elapsed_time_statistics_.set_enable(enabel);
  elapsed_time_statistics_.set_title("LoamAdvancedOdometry");
  std::cout << "display filter:" << std::endl;
  display_filter_->print_info();
}

void LoamAdvancedOdometry::set_extrinsic(const Eigen::Matrix4d & T_base_lidar)
{
  T_base_lidar_ = T_base_lidar;
  T_lidar_base_ = T_base_lidar.inverse();
}

bool LoamAdvancedOdometry::update(const slt_common::LidarData & lidar_data)
{
  elapsed_time_statistics_.tic("update");
  current_frame_.time = lidar_data.time;
  current_frame_.point_cloud = lidar_data.point_cloud;
  Eigen::Matrix4d final_pose;
  // extract feature
  feature_extraction_->extract(lidar_data.point_cloud, current_frame_.feature);
  if (key_frame_features_.empty()) {
    // initialize the first frame
    final_pose = T_base_lidar_;
  } else {
    // scan to map matching
    Eigen::Matrix4d predict_pose = Eigen::Matrix4d::Identity();
    if (!get_initial_pose_by_history(predict_pose)) {
      std::cout << "failed to get predict pose by history" << std::endl;
    }
    // downsample input feature for registration
    LoamAdvancedFeature input;
    input.edge = edge_input_filter_->apply(current_frame_.feature.edge);
    input.surf = surf_input_filter_->apply(current_frame_.feature.surf);
    registration_->match(input, predict_pose);
    final_pose = registration_->get_final_pose();
  }
  current_frame_.pose = final_pose;
  // add into history_poses_
  update_history_pose(current_frame_.time, final_pose);
  // update local map by key frame
  has_new_local_map_ = false;
  if (key_frame_features_.empty() || check_new_key_frame()) {
    update_local_map();
  }
  elapsed_time_statistics_.toc("update");
  elapsed_time_statistics_.print_all_info("update", 20);
  return true;
}

slt_common::OdomData LoamAdvancedOdometry::get_current_odom()
{
  slt_common::OdomData odom;
  odom.time = current_frame_.time;
  odom.pose = current_frame_.pose * T_lidar_base_;
  if (history_poses_.size() >= 2) {
    auto & pose1 = history_poses_[history_poses_.size() - 2];
    auto & pose2 = history_poses_[history_poses_.size() - 1];
    auto twist = slt_common::estimate_twist_by_pose(pose1, pose2);
    odom.linear_velocity = twist.linear_velocity;
    odom.angular_velocity = twist.angular_velocity;
  }
  return odom;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LoamAdvancedOdometry::get_current_scan()
{
  auto current_cloud = to_pointcloud_xyz(current_frame_.point_cloud);
  return display_filter_->apply(current_cloud);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr LoamAdvancedOdometry::get_feature_scan()
{
  return get_feature_point_cloud(current_frame_.feature, edge_rgb_, surf_rgb_);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr LoamAdvancedOdometry::get_local_map()
{
  LoamAdvancedFeature local_map;
  local_map.edge.reset(new pcl::PointCloud<pcl::PointXYZ>);
  local_map.surf.reset(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto & key_frame : key_frame_features_) {
    auto feature_in_map = transform_feature(key_frame.feature, key_frame.pose);
    *local_map.edge += *feature_in_map.edge;
    *local_map.surf += *feature_in_map.surf;
  }
  return get_feature_point_cloud(local_map, edge_rgb_, surf_rgb_);
}

bool LoamAdvancedOdometry::has_new_local_map() { return has_new_local_map_; }

bool LoamAdvancedOdometry::update_history_pose(double time, const Eigen::Matrix4d & pose)
{
  slt_common::PoseData pose_data;
  pose_data.time = time;
  pose_data.pose = pose;
  history_poses_.push_back(pose_data);
  if (history_poses_.size() > 100) {
    history_poses_.pop_front();
  }
  return true;
}

bool LoamAdvancedOdometry::get_initial_pose_by_history(Eigen::Matrix4d & initial_pose)
{
  if (history_poses_.empty()) {
    return false;
  }
  if (history_poses_.size() == 1) {
    initial_pose = history_poses_.back().pose;
  } else {
    Eigen::Matrix4d last_pose1 = history_poses_[history_poses_.size() - 2].pose;
    Eigen::Matrix4d last_pose2 = history_poses_.back().pose;
    Eigen::Matrix4d step_pose = last_pose1.inverse() * last_pose2;
    initial_pose = last_pose2 * step_pose;
  }
  return true;
}

bool LoamAdvancedOdometry::check_new_key_frame()
{
  Eigen::Vector3d dis = last_key_frame_pose_.block<3, 1>(0, 3) - current_frame_.pose.block<3, 1>(0, 3);
  if (dis.norm() > key_frame_distance_) {
    return true;
  }
  Eigen::Matrix3d R_rel =
    last_key_frame_pose_.block<3, 3>(0, 0).transpose() * current_frame_.pose.block<3, 3>(0, 0);
  if (Eigen::AngleAxisd(R_rel).angle() > key_frame_angle_) {
    return true;
  }
  return false;
}

bool LoamAdvancedOdometry::update_local_map()
{
  // add current key frame
  key_frame_features_.push_back(current_frame_);
  // move window for local map
  while (key_frame_features_.size() > static_cast<size_t>(local_frame_num_)) {
    key_frame_features_.pop_front();
  }
  last_key_frame_pose_ = current_frame_.pose;
  // rebuild local map, transform each key frame feature to map frame,
  // downsample after concat to remove duplicates across key frames
  LoamAdvancedFeature local_map;
  local_map.edge.reset(new pcl::PointCloud<pcl::PointXYZ>);
  local_map.surf.reset(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto & key_frame : key_frame_features_) {
    auto feature_in_map = transform_feature(key_frame.feature, key_frame.pose);
    *local_map.edge += *feature_in_map.edge;
    *local_map.surf += *feature_in_map.surf;
  }
  local_map.edge = edge_map_filter_->apply(local_map.edge);
  local_map.surf = surf_map_filter_->apply(local_map.surf);
  // rebuild kd tree
  registration_->set_target(local_map);
  has_new_local_map_ = true;
  return true;
}

}  // namespace slt_lidar_odometry
