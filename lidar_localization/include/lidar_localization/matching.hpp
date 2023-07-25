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
#include <memory>
#include <string>
//
#include "localization_common/cloud_filter/box_filter.hpp"
#include "localization_common/cloud_filter/cloud_filter_factory.hpp"
#include "localization_common/cloud_filter/cloud_filter_interface.hpp"
#include "localization_common/cloud_registration/cloud_registration_factory.hpp"
#include "localization_common/cloud_registration/cloud_registration_interface.hpp"
#include "localization_common/sensor_data/lidar_data.hpp"
#include "localization_common/sensor_data/odom_data.hpp"
#include "scan_context/scan_context_manager.hpp"

namespace lidar_localization
{
class Matching
{
public:
  Matching();
  bool init_config(const std::string & config_path, const std::string & data_path);
  // init pose
  bool set_init_pose_by_gnss(const Eigen::Matrix4d & init_pose);
  bool set_init_pose_by_scan_context(
    const localization_common::LidarData<pcl::PointXYZ> & init_scan);
  bool set_init_pose(const Eigen::Matrix4d & init_pose);
  Eigen::Matrix4d get_init_pose(void);
  bool has_inited();
  // update
  bool update(
    const localization_common::LidarData<pcl::PointXYZ> & lidar_data,
    Eigen::Matrix4d & cloud_pose);
  pcl::PointCloud<pcl::PointXYZ>::Ptr get_global_map();
  pcl::PointCloud<pcl::PointXYZ>::Ptr get_local_map();
  pcl::PointCloud<pcl::PointXYZ>::Ptr get_current_scan();
  bool has_new_global_map();
  bool has_new_local_map();

private:
  bool init_scan_context(const YAML::Node & config_node);
  bool init_global_map();
  bool reset_local_map(float x, float y, float z);

private:
  std::string data_path_ = "";
  std::string scan_context_path_ = "";
  std::string map_path_ = "";

  std::shared_ptr<scan_context::ScanContextManager> scan_context_manager_;
  std::shared_ptr<localization_common::CloudRegistrationInterface> registration_;

  std::shared_ptr<localization_common::CloudFilterInterface> global_map_filter_;

  std::shared_ptr<localization_common::BoxFilter> box_filter_;
  std::shared_ptr<localization_common::CloudFilterInterface> local_map_filter_;
  std::shared_ptr<localization_common::CloudFilterInterface> current_scan_filter_;
  //
  std::shared_ptr<localization_common::CloudRegistrationFactory> registration_factory_;
  std::shared_ptr<localization_common::CloudFilterFactory> cloud_filter_factory_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr current_scan_;

  Eigen::Matrix4d current_pose_ = Eigen::Matrix4d::Identity();

  Eigen::Matrix4d init_pose_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d current_gnss_pose_ = Eigen::Matrix4d::Identity();

  bool has_inited_ = false;
  bool has_new_global_map_ = false;
  bool has_new_local_map_ = false;
};
}  // namespace lidar_localization
