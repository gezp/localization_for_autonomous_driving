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

#include "lidar_odometry/lidar_odometry.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>

#include <filesystem>
#include <fstream>

namespace lidar_odometry
{
LidarOdometry::LidarOdometry()
: local_map_(new pcl::PointCloud<pcl::PointXYZ>())
{
  registration_factory_ = std::make_shared<localization_common::CloudRegistrationFactory>();
  cloud_filter_factory_ = std::make_shared<localization_common::CloudFilterFactory>();
}

bool LidarOdometry::init_config(const std::string & config_path)
{
  YAML::Node config_node = YAML::LoadFile(config_path);
  //
  local_frame_num_ = config_node["local_frame_num"].as<int>();
  key_frame_distance_ = config_node["key_frame_distance"].as<float>();
  // init registration and filter
  registration_ = registration_factory_->create(config_node);
  local_map_filter_ = cloud_filter_factory_->create(config_node["local_map_filter"]);
  current_scan_filter_ = cloud_filter_factory_->create(config_node["current_scan_filter"]);
  display_filter_ = cloud_filter_factory_->create(config_node["display_filter"]);
  // print info
  std::cout << "cloud registration:" << std::endl;
  registration_->print_info();
  std::cout << "local_map filter:" << std::endl;
  local_map_filter_->print_info();
  std::cout << "current_scan filter:" << std::endl;
  current_scan_filter_->print_info();
  std::cout << "display filter:" << std::endl;
  display_filter_->print_info();
  return true;
}

bool LidarOdometry::set_initial_pose(const Eigen::Matrix4d & initial_pose)
{
  initial_pose_ = initial_pose;
  return true;
}

bool LidarOdometry::update(const localization_common::LidarData<pcl::PointXYZ> & lidar_data)
{
  // reset param
  has_new_local_map_ = false;
  // remove invalid points
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*lidar_data.point_cloud, *lidar_data.point_cloud, indices);
  current_frame_.point_cloud = lidar_data.point_cloud;
  // downsample
  auto filtered_cloud = current_scan_filter_->apply(lidar_data.point_cloud);
  // get current pose
  if (key_frames_.empty()) {
    current_frame_.pose = initial_pose_;
  } else {
    // match
    Eigen::Matrix4d predict_pose = last_pose_ * step_pose_;
    registration_->match(filtered_cloud, predict_pose);
    current_frame_.pose = registration_->get_final_pose();
    // update step_pose
    step_pose_ = last_pose_.inverse() * current_frame_.pose;
  }
  last_pose_ = current_frame_.pose;
  // check if add new frame and update local map
  if (check_new_key_frame(current_frame_.pose)) {
    has_new_local_map_ = true;
    // add new key frame
    key_frames_.push_back(current_frame_);
    // move window for local map
    while (key_frames_.size() > static_cast<size_t>(local_frame_num_)) {
      key_frames_.pop_front();
    }
    // update
    update_local_map();
  }
  return true;
}

bool LidarOdometry::has_new_local_map() {return has_new_local_map_;}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarOdometry::get_local_map()
{
  return display_filter_->apply(local_map_);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarOdometry::get_current_scan()
{
  auto filtered_cloud = display_filter_->apply(current_frame_.point_cloud);
  pcl::transformPointCloud(*filtered_cloud, *filtered_cloud, current_frame_.pose);
  return filtered_cloud;
}

Eigen::Matrix4d LidarOdometry::get_current_pose() {return current_frame_.pose;}

bool LidarOdometry::check_new_key_frame(const Eigen::Matrix4d & pose)
{
  if (key_frames_.empty()) {
    return true;
  }
  Eigen::Vector3d dis = key_frames_.back().pose.block<3, 1>(0, 3) - pose.block<3, 1>(0, 3);
  if (dis.norm() > key_frame_distance_) {
    return true;
  }
  return false;
}

bool LidarOdometry::update_local_map()
{
  // update local map
  local_map_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  for (size_t i = 0; i < key_frames_.size(); ++i) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*key_frames_.at(i).point_cloud, *cloud, key_frames_.at(i).pose);
    *local_map_ += *cloud;
  }
  // update target of registration
  // 关键帧数量还比较少的时候不滤波，因为点云本来就不多，太稀疏影响匹配效果
  if (key_frames_.size() < 10) {
    registration_->set_target(local_map_);
  } else {
    auto filtered_local_map = local_map_filter_->apply(local_map_);
    registration_->set_target(filtered_local_map);
  }
  return true;
}

}  // namespace lidar_odometry
