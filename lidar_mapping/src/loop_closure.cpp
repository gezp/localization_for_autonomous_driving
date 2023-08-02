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
  scan_context_path_ = data_path_ + "/scan_context";
  // init_param
  extend_frame_num_ = config_node["extend_frame_num"].as<int>();
  loop_step_ = config_node["loop_step"].as<int>();
  diff_num_ = config_node["diff_num"].as<int>();
  detect_area_ = config_node["detect_area"].as<float>();
  fitness_score_limit_ = config_node["fitness_score_limit"].as<float>();
  // init key_frame_manager
  key_frame_manager_ = std::make_shared<localization_common::LidarKeyFrameManager>(data_path);
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

bool LoopClosure::reset(const std::vector<localization_common::LidarFrame> & key_frames)
{
  key_frame_manager_->reset(key_frames);
  return true;
}

bool LoopClosure::detect(const localization_common::LidarFrame & key_frame)
{
  // load back current scan and add into scan context
  auto current_scan = key_frame_manager_->load_point_cloud(key_frame.index);
  scan_context_manager_->update(current_scan, key_frame.pose.cast<float>());
  // only perform loop closure detection for every skip_num key frames:
  if (++skip_cnt_ < loop_step_) {
    return false;
  }
  // detect by scan context
  if (!scan_context_manager_->detect_loop_closure()) {
    return false;
  }
  int matched_index = scan_context_manager_->get_frame_index();
  // yaw change is unused, only output for debug
  double yaw_change_in_rad = scan_context_manager_->get_yaw_change();
  // check position difference:
  Eigen::Vector3d pos1 = key_frame.pose.block<3, 1>(0, 3);
  Eigen::Vector3d pos2 = key_frame_manager_->get_key_frame(matched_index).pose.block<3, 1>(0, 3);
  float key_frame_distance = (pos1 - pos2).head<2>().norm();
  if (key_frame_distance > detect_area_) {
    return false;
  }
  // build local map
  size_t start = matched_index - extend_frame_num_;
  size_t end = matched_index + extend_frame_num_;
  // this is needed for valid local map build
  if (matched_index < extend_frame_num_ || end >= key_frame_manager_->get_key_frame_count()) {
    return false;
  }
  auto filtered_map = key_frame_manager_->get_local_map(start, end, local_map_filter_);
  // scan to map registration
  registration_->set_target(filtered_map);
  registration_->match(current_scan_filter_->apply(current_scan), key_frame.pose);
  // check
  if (registration_->get_fitness_score() > fitness_score_limit_) {
    std::cout << "Loop-Closure detected " << current_loop_pose_.index0 << "<-->"
              << current_loop_pose_.index1 << std::endl
              << "drop due to high registration score: " << registration_->get_fitness_score()
              << std::endl;
    return false;
  }
  // current_loop_pose
  Eigen::Matrix4d relative_pose = key_frame_manager_->get_key_frame(matched_index).pose.inverse() *
    registration_->get_final_pose();
  current_loop_pose_.index0 = key_frame_manager_->get_key_frame(matched_index).index;
  current_loop_pose_.index1 = key_frame.index;
  current_loop_pose_.pose = relative_pose.cast<float>();
  //
  skip_cnt_ = 0;
  std::cout << "Loop-Closure detected " << current_loop_pose_.index0 << "<-->"
            << current_loop_pose_.index1 << std::endl
            << "scan context distance: " << scan_context_manager_->get_context_distance()
            << ", yaw change: " << yaw_change_in_rad << std::endl
            << "registration score: " << registration_->get_fitness_score() << std::endl;
  return true;
}

localization_common::LoopPose & LoopClosure::get_loop_pose() {return current_loop_pose_;}

bool LoopClosure::save()
{
  std::filesystem::remove_all(scan_context_path_);
  if (!std::filesystem::create_directory(scan_context_path_)) {
    return false;
  }
  return scan_context_manager_->save(scan_context_path_);
}

}  // namespace lidar_mapping
