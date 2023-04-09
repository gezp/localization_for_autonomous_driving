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

#include "lidar_mapping/loop_closure.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <filesystem>

namespace lidar_mapping
{
LoopClosure::LoopClosure()
{
  registration_factory_ = std::make_shared<localization_common::RegistrationFactory>();
  cloud_filter_factory_ = std::make_shared<localization_common::CloudFilterFactory>();
}

bool LoopClosure::init_config(const std::string & config_path, const std::string & data_path)
{
  YAML::Node config_node = YAML::LoadFile(config_path);
  // init_data_path
  data_path_ = data_path;
  key_frames_path_ = data_path_ + "/key_frames";
  scan_context_path_ = data_path_ + "/scan_context";
  // init_param
  extend_frame_num_ = config_node["extend_frame_num"].as<int>();
  loop_step_ = config_node["loop_step"].as<int>();
  diff_num_ = config_node["diff_num"].as<int>();
  detect_area_ = config_node["detect_area"].as<float>();
  fitness_score_limit_ = config_node["fitness_score_limit"].as<float>();
  //
  registration_ = registration_factory_->create(config_node);
  map_filter_ = cloud_filter_factory_->create(config_node["map_filter"]);
  current_scan_filter_ = cloud_filter_factory_->create(config_node["current_scan_filter"]);
  // get loop closure config:
  loop_closure_method_ = config_node["loop_closure_method"].as<std::string>();
  // create instance:
  scan_context_manager_ =
    std::make_shared<scan_context::ScanContextManager>(config_node[loop_closure_method_]);
  return true;
}

bool LoopClosure::update(
  const localization_common::CloudData & key_scan, const localization_common::KeyFrame & key_frame,
  const localization_common::KeyFrame & key_gnss)
{
  static int key_frame_index = 0;
  static float yaw_change_in_rad = 0.0f;

  has_new_loop_pose_ = false;

  scan_context_manager_->update(key_scan, key_gnss);

  all_key_frames_.push_back(key_frame);
  all_key_gnss_.push_back(key_gnss);

  if (!detect_nearest_key_frame(key_frame_index, yaw_change_in_rad)) {
    return false;
  }

  if (!align_cloud(key_frame_index, yaw_change_in_rad)) {
    return false;
  }

  has_new_loop_pose_ = true;
  return true;
}

bool LoopClosure::detect_nearest_key_frame(int & key_frame_index, float & yaw_change_in_rad)
{
  static int skip_cnt = 0;
  static int skip_num = loop_step_;

  // only perform loop closure detection for every skip_num key frames:
  if (++skip_cnt < skip_num) {
    return false;
  }

#ifndef SCAN_CONTEXT
  // generate loop-closure proposal using scan context match:
  std::pair<int, float> proposal = scan_context_manager_->detect_loop_closure();
  const int proposed_key_frame_id = proposal.first;
  const float proposed_yaw_change = proposal.second;

  // check proposal validity:
  if (scan_context::ScanContextManager::NONE == proposed_key_frame_id) {
    return false;
  }

  // check RTK position difference:
  const localization_common::KeyFrame & current_key_frame = all_key_gnss_.back();
  const localization_common::KeyFrame & proposed_key_frame =
    all_key_gnss_.at(proposed_key_frame_id);

  Eigen::Vector3f translation =
    (current_key_frame.pose.block<3, 1>(0, 3) - proposed_key_frame.pose.block<3, 1>(0, 3));
  float key_frame_distance = translation.norm();
#else
  // total number of GNSS/IMU key frame poses:
  const size_t N = all_key_gnss_.size();

  // ensure valid loop closure match:
  if (N < static_cast<size_t>(diff_num_ + 1)) {
    return false;
  }

  const localization_common::KeyFrame & current_key_frame = all_key_gnss_.back();

  int proposed_key_frame_id = scan_context::ScanContextManager::NONE;
  // this orientation compensation is not available for GNSS/IMU proposal:
  const float proposed_yaw_change = 0.0f;
  float key_frame_distance = std::numeric_limits<float>::max();
  for (size_t i = 0; i < N - 1; ++i) {
    // ensure key frame seq. distance:
    if (N < static_cast<size_t>(i + diff_num_)) {
      break;
    }

    const localization_common::KeyFrame & proposed_key_frame = all_key_gnss_.at(i);

    Eigen::Vector3f translation =
      (current_key_frame.pose.block<3, 1>(0, 3) - proposed_key_frame.pose.block<3, 1>(0, 3));
    float distance = translation.norm();

    // get closest proposal:
    if (distance < key_frame_distance) {
      key_frame_distance = distance;
      proposed_key_frame_id = i;
    }
  }
#endif

  // this is needed for valid local map build:
  if (proposed_key_frame_id < extend_frame_num_) {
    return false;
  }

  // update detection interval:
  skip_cnt = 0;
  skip_num = static_cast<int>(key_frame_distance);
  if (key_frame_distance > detect_area_) {
    skip_num = std::max(static_cast<int>(key_frame_distance / 2), loop_step_);
    return false;
  } else {
    key_frame_index = proposed_key_frame_id;
    yaw_change_in_rad = proposed_yaw_change;

    skip_num = loop_step_;
    return true;
  }
}

bool LoopClosure::align_cloud(const int key_frame_index, const float yaw_change_in_rad)
{
  // 生成地图
  localization_common::PointXYZCloudPtr map_cloud(new localization_common::PointXYZCloud());
  Eigen::Matrix4f map_pose = Eigen::Matrix4f::Identity();
  joint_map(key_frame_index, yaw_change_in_rad, map_cloud, map_pose);
  // 生成当前scan
  localization_common::PointXYZCloudPtr scan_cloud(new localization_common::PointXYZCloud());
  Eigen::Matrix4f scan_pose = Eigen::Matrix4f::Identity();
  joint_scan(scan_cloud, scan_pose);
  // 匹配
  Eigen::Matrix4f result_pose = Eigen::Matrix4f::Identity();
  localization_common::PointXYZCloudPtr result_cloud(new localization_common::PointXYZCloud());
  registration_->set_input_target(map_cloud);
  registration_->match(scan_cloud, scan_pose, result_cloud, result_pose);
  // 计算相对位姿
  current_loop_pose_.pose = map_pose.inverse() * result_pose;
  // 判断是否有效
  if (registration_->get_fitness_score() > fitness_score_limit_) {
    return false;
  }

  static int loop_close_cnt = 0;
  loop_close_cnt++;

  std::cout << std::endl
            << "Loop-Closure Detected " << current_loop_pose_.index0 << "<-->"
            << current_loop_pose_.index1 << std::endl
            << "\tFitness Score " << registration_->get_fitness_score() << std::endl
            << std::endl;

  return true;
}

bool LoopClosure::joint_map(
  const int key_frame_index, const float yaw_change_in_rad,
  localization_common::PointXYZCloudPtr & map_cloud, Eigen::Matrix4f & map_pose)
{
  // init map pose as loop closure pose:
  map_pose = all_key_gnss_.at(key_frame_index).pose;

  // apply yaw change estimation from scan context match:
  Eigen::AngleAxisf orientation_change(yaw_change_in_rad, Eigen::Vector3f::UnitZ());
  map_pose.block<3, 3>(0, 0) = map_pose.block<3, 3>(0, 0) * orientation_change.toRotationMatrix();

  current_loop_pose_.index0 = all_key_frames_.at(key_frame_index).index;

  // create local map:
  Eigen::Matrix4f pose_to_gnss = map_pose * all_key_frames_.at(key_frame_index).pose.inverse();
  for (int i = key_frame_index - extend_frame_num_; i < key_frame_index + extend_frame_num_; ++i) {
    // a. load back surrounding key scan:
    std::string file_path =
      key_frames_path_ + "/key_frame_" + std::to_string(all_key_frames_.at(i).index) + ".pcd";
    localization_common::PointXYZCloudPtr cloud(new localization_common::PointXYZCloud());
    pcl::io::loadPCDFile(file_path, *cloud);

    // b. transform surrounding key scan to map frame:
    Eigen::Matrix4f cloud_pose = pose_to_gnss * all_key_frames_.at(i).pose;
    pcl::transformPointCloud(*cloud, *cloud, cloud_pose);

    *map_cloud += *cloud;
  }
  // pre-process current map:
  map_filter_->filter(map_cloud, map_cloud);

  return true;
}

bool LoopClosure::joint_scan(
  localization_common::PointXYZCloudPtr & scan_cloud, Eigen::Matrix4f & scan_pose)
{
  // set scan pose as GNSS estimation:
  scan_pose = all_key_gnss_.back().pose;
  current_loop_pose_.index1 = all_key_frames_.back().index;
  current_loop_pose_.time = all_key_frames_.back().time;

  // load back current key scan:
  std::string file_path =
    key_frames_path_ + "/key_frame_" + std::to_string(all_key_frames_.back().index) + ".pcd";
  pcl::io::loadPCDFile(file_path, *scan_cloud);

  // pre-process current scan:
  current_scan_filter_->filter(scan_cloud, scan_cloud);

  return true;
}

bool LoopClosure::has_new_loop_pose() {return has_new_loop_pose_;}

localization_common::LoopPose & LoopClosure::get_current_loop_pose() {return current_loop_pose_;}

bool LoopClosure::save()
{
  std::filesystem::remove_all(scan_context_path_);
  if (!std::filesystem::create_directory(scan_context_path_)) {
    return false;
  }
  return scan_context_manager_->save(scan_context_path_);
}

}  // namespace lidar_mapping
