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

#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <deque>
#include <memory>
#include <vector>

#include "slt_common/point_cloud_filter/voxel_filter.hpp"
#include "slt_common/sensor_data/lidar_data.hpp"
#include "slt_common/sensor_data/odom_data.hpp"
#include "slt_common/sensor_data/pose_data.hpp"
#include "slt_common/tic_toc.hpp"
#include "slt_lidar_odometry/loam_advanced/loam_advanced_feature_extraction.hpp"
#include "slt_lidar_odometry/loam_advanced/loam_advanced_registration.hpp"

namespace slt_lidar_odometry
{

// scan to map loam odometry: extract edge/surf feature, match current scan
// against key frame based local map (sliding window), maintain local map
// by key frame distance & angle criterion.
class LoamAdvancedOdometry
{
  struct Frame
  {
    double time;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pcl::PointCloud<slt_common::PointXYZIRT>::Ptr point_cloud;
    LoamAdvancedFeature feature;
  };

public:
  explicit LoamAdvancedOdometry(const YAML::Node & config);
  void set_extrinsic(const Eigen::Matrix4d & T_base_lidar);
  bool update(const slt_common::LidarData & lidar_data);
  slt_common::OdomData get_current_odom();
  pcl::PointCloud<pcl::PointXYZ>::Ptr get_current_scan();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_feature_scan();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_local_map();
  bool has_new_local_map();

private:
  bool update_history_pose(double time, const Eigen::Matrix4d & pose);
  bool get_initial_pose_by_history(Eigen::Matrix4d & initial_pose);
  bool check_new_key_frame();
  bool update_local_map();

private:
  std::shared_ptr<slt_common::VoxelFilter> display_filter_;
  std::shared_ptr<LoamAdvancedFeatureExtraction> feature_extraction_;
  std::shared_ptr<LoamAdvancedRegistration> registration_;
  // input filters for registration
  std::shared_ptr<slt_common::VoxelFilter> edge_input_filter_;
  std::shared_ptr<slt_common::VoxelFilter> surf_input_filter_;
  // local map filters
  std::shared_ptr<slt_common::VoxelFilter> edge_map_filter_;
  std::shared_ptr<slt_common::VoxelFilter> surf_map_filter_;
  // tf
  Eigen::Matrix4d T_base_lidar_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_lidar_base_ = Eigen::Matrix4d::Identity();
  // data
  Frame current_frame_;
  std::deque<slt_common::PoseData> history_poses_;
  // key frame & local map
  std::deque<Frame> key_frame_features_;
  Eigen::Matrix4d last_key_frame_pose_ = Eigen::Matrix4d::Identity();
  int local_frame_num_{20};
  double key_frame_distance_{2.0};
  double key_frame_angle_{30.0};
  bool has_new_local_map_{false};
  // for rviz display
  std::vector<int> edge_rgb_{255, 0, 0};
  std::vector<int> surf_rgb_{0, 255, 0};
  // debug
  slt_common::AdvancedTicToc elapsed_time_statistics_;
};

}  // namespace slt_lidar_odometry
