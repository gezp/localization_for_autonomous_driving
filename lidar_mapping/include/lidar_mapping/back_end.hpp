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
#include <vector>
#include <string>

#include "localization_common/sensor_data/lidar_data.hpp"
#include "localization_common/sensor_data/loop_candidate.hpp"
#include "localization_common/sensor_data/odom_data.hpp"
#include "localization_common/cloud_filter/voxel_filter.hpp"
#include "localization_common/lidar_key_frame_manager.hpp"
#include "localization_common/odom_data_buffer.hpp"
#include "lidar_mapping/graph_optimizer/g2o_graph_optimizer.hpp"

namespace lidar_mapping
{

struct GraphOptimizerConfig
{
  bool use_gnss = true;
  bool use_loop_close = false;

  Eigen::Matrix<double, 6, 1> odom_edge_noise;
  Eigen::Matrix<double, 6, 1> close_loop_noise;
  Eigen::Vector3d gnss_noise;

  int optimize_step_with_key_frame = 100;
  int optimize_step_with_gnss = 100;
  int optimize_step_with_loop = 10;
};

class BackEnd
{
public:
  BackEnd() = default;
  ~BackEnd() = default;
  bool init_config(const std::string & config_path, const std::string & data_path);
  void set_extrinsic(const Eigen::Matrix4d & T_base_lidar);
  bool add_gnss_odom(const localization_common::OdomData & gnss_odom);
  bool add_loop_candidate(const localization_common::LoopCandidate & loop_candidate);
  bool update(
    const localization_common::LidarData<pcl::PointXYZ> & lidar_data,
    const localization_common::OdomData & lidar_odom);
  bool optimize(bool force = true);
  bool has_new_key_frame();
  bool has_new_optimized();
  localization_common::OdomData get_current_odom();
  const std::vector<localization_common::LidarFrame> & get_key_frames();
  pcl::PointCloud<pcl::PointXYZ>::Ptr get_global_map();
  bool save_map();

private:
  bool init_graph_optimizer(const YAML::Node & config_node);
  bool check_new_key_frame(const localization_common::OdomData & lidar_odom);
  bool add_node_and_edge();

private:
  std::shared_ptr<localization_common::LidarKeyFrameManager> key_frame_manager_;
  std::shared_ptr<localization_common::VoxelFilter> display_filter_;
  std::shared_ptr<localization_common::VoxelFilter> global_map_filter_;
  // optimizer
  GraphOptimizerConfig graph_optimizer_config_;
  std::shared_ptr<GraphOptimizerInterface> graph_optimizer_;
  // data
  std::shared_ptr<localization_common::OdomDataBuffer> gnss_odom_buffer_;
  localization_common::OdomData current_lidar_odom_;
  localization_common::OdomData latest_key_lidar_odom_;

  Eigen::Matrix4d T_map_odom_ = Eigen::Matrix4d::Identity();

  float key_frame_distance_ = 2.0;
  int new_gnss_cnt_ = 0;
  int new_loop_cnt_ = 0;
  int new_key_frame_cnt_ = 0;
  bool has_new_key_frame_ = false;
  bool has_new_optimized_ = false;
  //
  Eigen::Matrix4d T_base_lidar_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_lidar_base_ = Eigen::Matrix4d::Identity();
};
}  // namespace lidar_mapping
