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

#include "lidar_mapping/front_end.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>

#include <filesystem>
#include <fstream>

namespace lidar_mapping
{
FrontEnd::FrontEnd()
: local_map_(new localization_common::PointXYZCloud())
{
  registration_factory_ = std::make_shared<localization_common::RegistrationFactory>();
  cloud_filter_factory_ = std::make_shared<localization_common::CloudFilterFactory>();
}

bool FrontEnd::init_config(const std::string & config_path)
{
  YAML::Node config_node = YAML::LoadFile(config_path);
  key_frame_distance_ = config_node["key_frame_distance"].as<float>();
  local_frame_num_ = config_node["local_frame_num"].as<int>();
  registration_ = registration_factory_->create(config_node);
  local_map_filter_ = cloud_filter_factory_->create(config_node["local_map_filter"]);
  current_scan_filter_ = cloud_filter_factory_->create(config_node["current_scan_filter"]);

  return true;
}

bool FrontEnd::set_init_pose(const Eigen::Matrix4f & init_pose)
{
  init_pose_ = init_pose;
  return true;
}

bool FrontEnd::update(
  const localization_common::CloudData & cloud_data, Eigen::Matrix4f & cloud_pose)
{
  static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
  static Eigen::Matrix4f last_pose = init_pose_;
  static Eigen::Matrix4f predict_pose = init_pose_;
  static Eigen::Matrix4f last_key_frame_pose = init_pose_;

  //
  // set up current scan:
  //
  current_frame_.cloud_data.time = cloud_data.time;
  // a. remove invalid measurements:
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_data.cloud, *current_frame_.cloud_data.cloud, indices);
  // b. apply filter to current scan:
  localization_common::PointXYZCloudPtr filtered_cloud(new localization_common::PointXYZCloud());
  current_scan_filter_->filter(current_frame_.cloud_data.cloud, filtered_cloud);

  //
  // set up local map:
  //
  if (local_map_frames_.size() == 0) {
    current_frame_.pose = init_pose_;
    update_with_new_frame(current_frame_);
    cloud_pose = current_frame_.pose;
    return true;
  }

  //
  // update lidar odometry using scan match result:
  //
  localization_common::PointXYZCloudPtr result_cloud(new localization_common::PointXYZCloud());
  registration_->match(filtered_cloud, predict_pose, result_cloud, current_frame_.pose);
  cloud_pose = current_frame_.pose;

  //
  // update init pose for next scan match:
  //
  step_pose = last_pose.inverse() * current_frame_.pose;
  predict_pose = current_frame_.pose * step_pose;
  last_pose = current_frame_.pose;

  //
  // shall the key frame set be updated:
  //
  if (
    fabs(last_key_frame_pose(0, 3) - current_frame_.pose(0, 3)) +
    fabs(last_key_frame_pose(1, 3) - current_frame_.pose(1, 3)) +
    fabs(last_key_frame_pose(2, 3) - current_frame_.pose(2, 3)) >
    key_frame_distance_)
  {
    update_with_new_frame(current_frame_);
    last_key_frame_pose = current_frame_.pose;
  }

  return true;
}

bool FrontEnd::update_with_new_frame(const Frame & new_key_frame)
{
  Frame key_frame = new_key_frame;
  // 这一步的目的是为了把关键帧的点云保存下来
  // 由于用的是共享指针，所以直接复制只是复制了一个指针而已
  // 此时无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云
  key_frame.cloud_data.cloud.reset(
    new localization_common::PointXYZCloud(*new_key_frame.cloud_data.cloud));

  // keep only the latest local_frame_num_ frames:
  local_map_frames_.push_back(key_frame);
  while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_)) {
    local_map_frames_.pop_front();
  }

  // transform all local frame measurements to map frame
  // to create local map:
  local_map_.reset(new localization_common::PointXYZCloud());
  localization_common::PointXYZCloudPtr transformed_cloud(new localization_common::PointXYZCloud());
  for (size_t i = 0; i < local_map_frames_.size(); ++i) {
    pcl::transformPointCloud(
      *local_map_frames_.at(i).cloud_data.cloud, *transformed_cloud, local_map_frames_.at(i).pose);

    *local_map_ += *transformed_cloud;
  }

  // scan-to-map matching:
  // set target as local map:
  if (local_map_frames_.size() < 10) {
    registration_->set_input_target(local_map_);
  } else {
    localization_common::PointXYZCloudPtr filtered_local_map_ptr(
      new localization_common::PointXYZCloud());
    local_map_filter_->filter(local_map_, filtered_local_map_ptr);
    registration_->set_input_target(filtered_local_map_ptr);
  }

  return true;
}
}  // namespace lidar_mapping
