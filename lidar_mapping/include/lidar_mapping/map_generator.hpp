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
#include <string>
#include <deque>
#include <memory>

#include "localization_common/cloud_filter/cloud_filter_factory.hpp"
#include "localization_common/sensor_data/lidar_data.hpp"
#include "localization_common/sensor_data/key_frame.hpp"
#include "localization_common/sensor_data/odom_data.hpp"

namespace lidar_mapping
{
class MapGenerator
{
public:
  MapGenerator();
  bool init_config(const std::string & config_path, const std::string & data_path);
  pcl::PointCloud<pcl::PointXYZ>::Ptr joint_cloud_map(
    const std::deque<localization_common::KeyFrame> & key_frames);
  pcl::PointCloud<pcl::PointXYZ>::Ptr get_global_map(
    const std::deque<localization_common::KeyFrame> & optimized_key_frames,
    bool use_display_filter = true);
  bool save_map(const std::deque<localization_common::KeyFrame> & optimized_key_frames);

private:
  std::string data_path_ = "";
  std::string key_frames_path_ = "";
  std::string map_path_ = "";

  std::shared_ptr<localization_common::CloudFilterInterface> display_filter_;
  std::shared_ptr<localization_common::CloudFilterInterface> global_map_filter_;
  std::shared_ptr<localization_common::CloudFilterFactory> cloud_filter_factory_;
};
}  // namespace lidar_mapping
