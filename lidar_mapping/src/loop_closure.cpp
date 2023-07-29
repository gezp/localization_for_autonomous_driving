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
  registration_factory_ = std::make_shared<localization_common::CloudRegistrationFactory>();
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
  local_map_filter_ = cloud_filter_factory_->create(config_node["local_map_filter"]);
  current_scan_filter_ = cloud_filter_factory_->create(config_node["current_scan_filter"]);
  // get loop closure config:
  loop_closure_method_ = config_node["loop_closure_method"].as<std::string>();
  // create instance:
  scan_context_manager_ =
    std::make_shared<scan_context::ScanContextManager>(config_node[loop_closure_method_]);
  // print info
  std::cout << "point cloud registration:" << std::endl;
  registration_->print_info();
  std::cout << "local_map filter:" << std::endl;
  local_map_filter_->print_info();
  std::cout << "current_scan filter:" << std::endl;
  current_scan_filter_->print_info();
  return true;
}

bool LoopClosure::update(const localization_common::KeyFrame & key_frame)
{
  has_new_loop_pose_ = false;
  all_key_frames_.push_back(key_frame);
  // load back current scan:
  pcl::PointCloud<pcl::PointXYZ>::Ptr current_scan(new pcl::PointCloud<pcl::PointXYZ>());
  std::string file_path =
    key_frames_path_ + "/key_frame_" + std::to_string(all_key_frames_.back().index) + ".pcd";
  pcl::io::loadPCDFile(file_path, *current_scan);
  // add into scan context
  scan_context_manager_->update(current_scan, key_frame.pose);
  // detect
  int key_frame_index = 0;
  float yaw_change_in_rad = 0.0f;
  if (!detect_nearest_key_frame(key_frame_index, yaw_change_in_rad)) {
    return false;
  }
  // local map
  auto map = joint_map(key_frame_index);
  auto filtered_map = local_map_filter_->apply(map);
  // current scan
  auto filtered_scan = current_scan_filter_->apply(current_scan);
  // scan to map registration
  registration_->set_target(filtered_map);
  registration_->match(filtered_scan, all_key_frames_.back().pose.cast<double>());
  auto result_pose = registration_->get_final_pose().cast<float>();
  // current_loop_pose
  current_loop_pose_.index0 = all_key_frames_.back().index;
  current_loop_pose_.index1 = all_key_frames_.at(key_frame_index).index;
  current_loop_pose_.time = all_key_frames_.back().time;
  current_loop_pose_.pose = all_key_frames_.back().pose.inverse() * result_pose;
  //
  std::cout << "Loop-Closure detected " << current_loop_pose_.index0 << "<-->"
            << current_loop_pose_.index1 << std::endl
            << "scan context distance: " << scan_context_manager_->get_context_distance()
            << ", registration score: " << registration_->get_fitness_score() << std::endl;
  has_new_loop_pose_ = true;
  return true;
}

bool LoopClosure::detect_nearest_key_frame(int & key_frame_index, float & yaw_change_in_rad)
{
  static int skip_cnt = 0;

  // only perform loop closure detection for every skip_num key frames:
  if (++skip_cnt < loop_step_) {
    return false;
  }

  // generate loop-closure proposal using scan context match:
  if (!scan_context_manager_->detect_loop_closure()) {
    return false;
  }
  int proposed_key_frame_id = scan_context_manager_->get_frame_index();
  double proposed_yaw_change = scan_context_manager_->get_yaw_change();

  // this is needed for valid local map build
  if (proposed_key_frame_id < extend_frame_num_) {
    return false;
  }
  // check position difference:
  auto & current_key_frame = all_key_frames_.back();
  auto & proposed_key_frame = all_key_frames_.at(proposed_key_frame_id);
  Eigen::Vector3f translation =
    (current_key_frame.pose.block<3, 1>(0, 3) - proposed_key_frame.pose.block<3, 1>(0, 3));
  float key_frame_distance = translation.head<2>().norm();
  if (key_frame_distance > detect_area_) {
    return false;
  }
  //
  key_frame_index = proposed_key_frame_id;
  yaw_change_in_rad = proposed_yaw_change;
  skip_cnt = 0;
  return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LoopClosure::joint_map(int key_frame_index)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (int i = key_frame_index - extend_frame_num_; i < key_frame_index + extend_frame_num_; ++i) {
    // a. load back surrounding key scan:
    std::string file_path =
      key_frames_path_ + "/key_frame_" + std::to_string(all_key_frames_.at(i).index) + ".pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::io::loadPCDFile(file_path, *cloud);
    // b. transform surrounding key scan to map frame:
    pcl::transformPointCloud(*cloud, *cloud, all_key_frames_.at(i).pose);
    *map_cloud += *cloud;
  }
  return map_cloud;
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
