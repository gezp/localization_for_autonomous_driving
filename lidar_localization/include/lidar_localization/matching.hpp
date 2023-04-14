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
#include "localization_common/registration/registration_factory.hpp"
#include "localization_common/registration/registration_interface.hpp"
#include "localization_common/sensor_data/cloud_data.hpp"
#include "localization_common/sensor_data/pose_data.hpp"
#include "scan_context/scan_context_manager.hpp"

namespace lidar_localization
{
class Matching
{
public:
  Matching();
  bool init_config(const std::string & config_path, const std::string & data_path);
  // init pose
  bool set_init_pose_by_gnss(const Eigen::Matrix4f & init_pose);
  bool set_init_pose_by_scan_context(const localization_common::CloudData & init_scan);
  bool set_init_pose(const Eigen::Matrix4f & init_pose);
  Eigen::Matrix4f get_init_pose(void);
  bool has_inited();
  // update
  bool update(const localization_common::CloudData & cloud_data, Eigen::Matrix4f & cloud_pose);
  localization_common::PointXYZCloudPtr get_global_map();
  localization_common::PointXYZCloudPtr get_local_map();
  localization_common::PointXYZCloudPtr get_current_scan();
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
  std::shared_ptr<localization_common::RegistrationInterface> registration_;

  std::shared_ptr<localization_common::CloudFilterInterface> global_map_filter_;

  std::shared_ptr<localization_common::BoxFilter> box_filter_;
  std::shared_ptr<localization_common::CloudFilterInterface> local_map_filter_;
  std::shared_ptr<localization_common::CloudFilterInterface> current_scan_filter_;
  //
  std::shared_ptr<localization_common::RegistrationFactory> registration_factory_;
  std::shared_ptr<localization_common::CloudFilterFactory> cloud_filter_factory_;

  localization_common::PointXYZCloudPtr global_map_;
  localization_common::PointXYZCloudPtr local_map_;
  localization_common::PointXYZCloudPtr current_scan_;

  Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();

  Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f current_gnss_pose_ = Eigen::Matrix4f::Identity();

  bool has_inited_ = false;
  bool has_new_global_map_ = false;
  bool has_new_local_map_ = false;
};
}  // namespace lidar_localization
