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
#include "localization_common/sensor_data/cloud_data.hpp"
#include "localization_common/cloud_filter/cloud_filter_factory.hpp"
#include "localization_common/cloud_registration/cloud_registration_factory.hpp"

namespace lidar_odometry
{
class FrontEnd
{
public:
  struct Frame
  {
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    localization_common::CloudData cloud_data;
  };

public:
  FrontEnd();
  bool init_config(const std::string & config_path);
  bool set_init_pose(const Eigen::Matrix4f & init_pose);
  bool update(const localization_common::CloudData & cloud_data, Eigen::Matrix4f & cloud_pose);
  bool has_new_local_map();
  localization_common::PointXYZCloudPtr get_local_map();
  localization_common::PointXYZCloudPtr get_current_scan();

private:
  bool update_with_new_frame(const Frame & new_key_frame);

private:
  std::shared_ptr<localization_common::CloudFilterInterface> current_scan_filter_;
  std::shared_ptr<localization_common::CloudFilterInterface> local_map_filter_;
  std::shared_ptr<localization_common::CloudFilterInterface> display_filter_;
  std::shared_ptr<localization_common::CloudRegistrationInterface> registration_;
  std::shared_ptr<localization_common::CloudRegistrationFactory> registration_factory_;
  std::shared_ptr<localization_common::CloudFilterFactory> cloud_filter_factory_;
  std::deque<Frame> local_map_frames_;

  bool has_new_local_map_ = false;
  localization_common::PointXYZCloudPtr local_map_;
  localization_common::PointXYZCloudPtr result_cloud_;
  Frame current_frame_;

  Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

  float key_frame_distance_ = 2.0;
  int local_frame_num_ = 20;
};

}  // namespace lidar_odometry
