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

#pragma once

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <deque>
#include <memory>

#include "localization_common/sensor_data/lidar_data.hpp"
#include "localization_common/sensor_data/odom_data.hpp"
#include "localization_common/sensor_data/pose_data.hpp"
#include "localization_common/cloud_filter/voxel_filter.hpp"
#include "localization_common/cloud_registration/cloud_registration_factory.hpp"

namespace lidar_odometry
{

class LidarOdometry
{
  struct Frame
  {
    double time;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;
  };

public:
  explicit LidarOdometry(const YAML::Node & config);
  ~LidarOdometry() = default;
  void set_extrinsic(const Eigen::Matrix4d & T_base_lidar);
  void set_initial_pose(const Eigen::Matrix4d & initial_pose);
  bool update(const localization_common::LidarData<pcl::PointXYZ> & lidar_data);
  localization_common::OdomData get_current_odom();
  pcl::PointCloud<pcl::PointXYZ>::Ptr get_current_scan();
  pcl::PointCloud<pcl::PointXYZ>::Ptr get_local_map();
  bool has_new_local_map();

private:
  bool update_history_pose(double time, const Eigen::Matrix4d & pose);
  bool get_initial_pose_by_history(Eigen::Matrix4d & initial_pose);
  bool check_new_key_frame(const Eigen::Matrix4d & pose);
  bool update_local_map();
  bool match_scan_to_map(const Eigen::Matrix4d & predict_pose, Eigen::Matrix4d & final_pose);

private:
  std::shared_ptr<localization_common::CloudRegistrationFactory> registration_factory_;
  std::shared_ptr<localization_common::CloudRegistrationInterface> registration_;
  std::shared_ptr<localization_common::VoxelFilter> current_scan_filter_;
  std::shared_ptr<localization_common::VoxelFilter> local_map_filter_;
  std::shared_ptr<localization_common::VoxelFilter> display_filter_;
  // params for local map
  float key_frame_distance_ = 2.0;
  int local_frame_num_ = 20;
  // tf
  Eigen::Matrix4d T_base_lidar_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_lidar_base_ = Eigen::Matrix4d::Identity();
  // data
  Eigen::Matrix4d initial_pose_ = Eigen::Matrix4d::Identity();
  Frame current_frame_;
  std::deque<localization_common::PoseData> history_poses_;
  std::deque<Frame> key_frames_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_;
  bool has_new_local_map_ = false;
};

}  // namespace lidar_odometry
