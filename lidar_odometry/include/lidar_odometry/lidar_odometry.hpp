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
#include <string>
#include <memory>
//
#include "localization_common/sensor_data/lidar_data.hpp"
#include "localization_common/cloud_filter/cloud_filter_factory.hpp"
#include "localization_common/cloud_registration/cloud_registration_factory.hpp"

namespace lidar_odometry
{
class LidarOdometry
{
  struct Frame
  {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;
  };

public:
  LidarOdometry();
  bool init_config(const std::string & config_path);
  void set_initial_pose(const Eigen::Matrix4d & initial_pose);
  void set_extrinsic(const Eigen::Matrix4d & T_base_lidar);
  bool update(const localization_common::LidarData<pcl::PointXYZ> & lidar_data);
  bool has_new_local_map();
  pcl::PointCloud<pcl::PointXYZ>::Ptr get_local_map();
  pcl::PointCloud<pcl::PointXYZ>::Ptr get_current_scan();
  Eigen::Matrix4d get_current_pose();

private:
  bool check_new_key_frame(const Eigen::Matrix4d & pose);
  bool update_local_map();

private:
  std::shared_ptr<localization_common::CloudFilterInterface> current_scan_filter_;
  std::shared_ptr<localization_common::CloudFilterInterface> local_map_filter_;
  std::shared_ptr<localization_common::CloudFilterInterface> display_filter_;
  std::shared_ptr<localization_common::CloudRegistrationInterface> registration_;
  std::shared_ptr<localization_common::CloudRegistrationFactory> registration_factory_;
  std::shared_ptr<localization_common::CloudFilterFactory> cloud_filter_factory_;
  //
  Eigen::Matrix4d T_base_lidar_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_lidar_base_ = Eigen::Matrix4d::Identity();
  // data
  std::deque<Frame> key_frames_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_;
  Frame current_frame_;
  Eigen::Matrix4d last_pose_;
  Eigen::Matrix4d initial_pose_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d step_pose_ = Eigen::Matrix4d::Identity();

  float key_frame_distance_ = 2.0;
  int local_frame_num_ = 20;
  bool has_new_local_map_ = false;
};

}  // namespace lidar_odometry
