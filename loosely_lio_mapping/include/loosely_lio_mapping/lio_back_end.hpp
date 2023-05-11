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

#include <memory>
#include <deque>
#include <fstream>
#include <string>

#include "localization_common/sensor_data/cloud_data.hpp"
#include "localization_common/sensor_data/imu_data.hpp"
#include "localization_common/sensor_data/key_frame.hpp"
#include "localization_common/sensor_data/loop_pose.hpp"
#include "localization_common/sensor_data/pose_data.hpp"
#include "localization_common/sensor_data/velocity_data.hpp"
#include "loosely_lio_mapping/graph_optimizer/g2o_graph_optimizer.hpp"
#include "loosely_lio_mapping/graph_optimizer/imu_pre_integration.hpp"

namespace loosely_lio_mapping
{

class LioBackEnd
{
public:
  LioBackEnd();
  bool init_config(const std::string & config_path, const std::string & data_path);
  void set_imu_extrinsic(const Eigen::Matrix4f & T_base_imu);
  bool update(
    const localization_common::CloudData & cloud_data,
    const localization_common::PoseData & lidar_odom,
    const localization_common::PoseData & gnss_pose, const localization_common::IMUData & imu_data);
  bool insert_loop_pose(const localization_common::LoopPose & loop_pose);
  bool add_raw_imu(const localization_common::IMUData & imu_data);

  bool optimize();

  bool has_new_key_frame();
  bool has_new_optimized();
  void get_latest_key_scan(localization_common::CloudData & key_scan);
  void get_latest_key_frame(localization_common::KeyFrame & key_frame);
  void get_latest_key_gnss(localization_common::KeyFrame & key_frame);
  std::deque<localization_common::KeyFrame> get_optimized_key_frames();
  Eigen::Matrix4f get_odom_to_map();

private:
  bool init_graph_optimizer(const YAML::Node & config_node);
  bool add_new_key_frame(
    const localization_common::CloudData & cloud_data,
    const localization_common::PoseData & lidar_odom,
    const localization_common::PoseData & gnss_pose);
  bool add_node_and_edge();
  bool check_new_key_frame(const localization_common::PoseData & lidar_odom);
  bool check_need_optimize();

private:
  std::string data_path_ = "";
  std::string key_frames_path_ = "";

  float key_frame_distance_ = 2.0;

  bool has_new_key_frame_ = false;
  bool has_new_optimized_ = false;

  localization_common::CloudData current_key_scan_;
  localization_common::KeyFrame current_key_frame_;
  localization_common::KeyFrame current_key_gnss_;
  // raw key frames and optimized key frames
  std::deque<localization_common::KeyFrame> key_frames_;
  std::deque<localization_common::KeyFrame> optimized_key_frames_;
  //
  Eigen::Matrix4f pose_to_optimize_ = Eigen::Matrix4f::Identity();
  // pre-integrator:
  double gravity_magnitude_;

  // optimizer:
  std::shared_ptr<GraphOptimizerInterface> graph_optimizer_;
  Eigen::Matrix4f T_base_imu_ = Eigen::Matrix4f::Identity();

  // params
  bool use_gnss_ = true;
  bool use_loop_closure_ = false;
  bool use_imu_pre_integration_ = false;
  struct
  {
    int key_frame = 100;
    int loop_closure = 10;
  } optimization_step_size_;

  Eigen::VectorXd lidar_odom_noise_;
  Eigen::VectorXd loop_closure_noise_;
  Eigen::VectorXd gnss_noise_;

  int new_gnss_cnt_ = 0;
  int new_loop_cnt_ = 0;
  int new_key_frame_cnt_ = 0;
};

}  // namespace loosely_lio_mapping
