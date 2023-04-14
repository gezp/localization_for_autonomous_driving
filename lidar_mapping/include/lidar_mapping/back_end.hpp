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
#include "localization_common/sensor_data/key_frame.hpp"
#include "localization_common/sensor_data/loop_pose.hpp"
#include "localization_common/sensor_data/pose_data.hpp"
#include "lidar_mapping/graph_optimizer/g2o_graph_optimizer.hpp"
#include "lidar_mapping/map_generator.hpp"

namespace lidar_mapping
{
class BackEnd
{
public:
  BackEnd();
  bool init_config(const std::string & config_path, const std::string & data_path);
  bool update(
    const localization_common::CloudData & cloud_data,
    const localization_common::PoseData & lidar_odom,
    const localization_common::PoseData & gnss_pose);
  bool insert_loop_pose(const localization_common::LoopPose & loop_pose);
  bool optimize();
  //
  bool has_new_key_frame();
  bool has_new_optimized();
  //
  void get_latest_key_scan(localization_common::CloudData & key_scan);
  void get_latest_key_frame(localization_common::KeyFrame & key_frame);
  void get_latest_key_gnss(localization_common::KeyFrame & key_frame);
  //
  std::deque<localization_common::KeyFrame> get_optimized_key_frames();
  Eigen::Matrix4f get_lidar_odom_to_map();
  localization_common::PointXYZCloudPtr get_global_map();
  bool save_map();

private:
  bool init_graph_optimizer(const YAML::Node & config_node);
  bool add_new_key_frame(
    const localization_common::CloudData & cloud_data,
    const localization_common::PoseData & lidar_odom,
    const localization_common::PoseData & gnss_pose);
  bool add_node_and_edge(const localization_common::PoseData & gnss_data);
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
  // 优化器
  std::shared_ptr<GraphOptimizerInterface> graph_optimizer_;

  class GraphOptimizerConfig
  {
public:
    GraphOptimizerConfig()
    {
      odom_edge_noise.resize(6);
      close_loop_noise.resize(6);
      gnss_noise.resize(3);
    }

public:
    bool use_gnss = true;
    bool use_loop_close = false;

    Eigen::VectorXd odom_edge_noise;
    Eigen::VectorXd close_loop_noise;
    Eigen::VectorXd gnss_noise;

    int optimize_step_with_key_frame = 100;
    int optimize_step_with_gnss = 100;
    int optimize_step_with_loop = 10;
  };
  GraphOptimizerConfig graph_optimizer_config_;

  int new_gnss_cnt_ = 0;
  int new_loop_cnt_ = 0;
  int new_key_frame_cnt_ = 0;
  //
  std::shared_ptr<MapGenerator> map_generator_;
};
}  // namespace lidar_mapping
