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

#include <pcl/registration/ndt.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <deque>
#include <memory>
#include <string>
//
#include "localization_common/cloud_filter/cloud_filter_factory.hpp"
#include "localization_common/registration/registration_factory.hpp"
#include "localization_common/sensor_data/key_frame.hpp"
#include "localization_common/sensor_data/loop_pose.hpp"
#include "scan_context/scan_context_manager.hpp"

namespace lidar_mapping
{
class LoopClosure
{
public:
  LoopClosure();
  bool init_config(const std::string & config_path, const std::string & data_path);
  bool update(
    const localization_common::CloudData & key_scan,
    const localization_common::KeyFrame & key_frame,
    const localization_common::KeyFrame & key_gnss);

  bool has_new_loop_pose();
  localization_common::LoopPose & get_current_loop_pose();
  bool save(void);

private:
  bool DetectNearestKeyFrame(int & key_frame_index, float & yaw_change_in_rad);
  bool align_cloud(const int key_frame_index, const float yaw_change_in_rad);
  bool joint_map(
    const int key_frame_index, const float yaw_change_in_rad,
    localization_common::PointXYZCloudPtr & map_cloud, Eigen::Matrix4f & map_pose);
  bool joint_scan(localization_common::PointXYZCloudPtr & scan_cloud, Eigen::Matrix4f & scan_pose);

private:
  std::string data_path_ = "";
  std::string key_frames_path_ = "";
  std::string scan_context_path_ = "";

  std::string loop_closure_method_ = "";

  int extend_frame_num_ = 3;
  int loop_step_ = 10;
  int diff_num_ = 100;
  float detect_area_ = 10.0;
  float fitness_score_limit_ = 2.0;

  std::shared_ptr<localization_common::CloudFilterInterface> current_scan_filter_;
  std::shared_ptr<localization_common::CloudFilterInterface> map_filter_;
  std::shared_ptr<localization_common::RegistrationInterface> registration_;
  //
  std::shared_ptr<localization_common::RegistrationFactory> registration_factory_;
  std::shared_ptr<localization_common::CloudFilterFactory> cloud_filter_factory_;
  //
  std::shared_ptr<scan_context::ScanContextManager> scan_context_manager_;

  std::deque<localization_common::KeyFrame> all_key_frames_;
  std::deque<localization_common::KeyFrame> all_key_gnss_;

  localization_common::LoopPose current_loop_pose_;
  bool has_new_loop_pose_ = false;
};
}  // namespace lidar_mapping
