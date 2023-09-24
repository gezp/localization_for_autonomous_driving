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
#include <string>
#include <vector>
#include <deque>

#include "localization_common/sensor_data/lidar_data.hpp"
#include "localization_common/sensor_data/imu_data.hpp"
#include "localization_common/sensor_data/lidar_frame.hpp"
#include "localization_common/sensor_data/loop_candidate.hpp"
#include "localization_common/sensor_data/odom_data.hpp"
#include "localization_common/cloud_filter/cloud_filter_factory.hpp"
#include "localization_common/lidar_key_frame_manager.hpp"
#include "localization_common/odom_data_buffer.hpp"
#include "loosely_lio_mapping/graph_optimizer/g2o_graph_optimizer.hpp"

namespace loosely_lio_mapping
{

class LioBackEnd
{
public:
  LioBackEnd();
  bool init_config(const std::string & config_path, const std::string & data_path);
  void set_extrinsic(const Eigen::Matrix4d & T_base_imu, const Eigen::Matrix4d & T_lidar_imu);
  bool add_imu_data(const localization_common::ImuData & imu);
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
  bool get_synced_imu_buffer(double time, std::vector<localization_common::ImuData> & buffer);
  bool add_node_and_edge();

private:
  std::shared_ptr<localization_common::CloudFilterFactory> cloud_filter_factory_;
  std::shared_ptr<localization_common::CloudFilterInterface> display_filter_;
  std::shared_ptr<localization_common::CloudFilterInterface> global_map_filter_;
  // key frame manager
  std::shared_ptr<localization_common::LidarKeyFrameManager> key_frame_manager_;
  // optimizer
  std::shared_ptr<GraphOptimizerInterface> graph_optimizer_;
  // data
  std::deque<localization_common::ImuData> imu_buffer_;
  std::shared_ptr<localization_common::OdomDataBuffer> gnss_odom_buffer_;
  localization_common::OdomData current_lidar_odom_;
  localization_common::OdomData latest_key_lidar_odom_;
  localization_common::ImuData lastest_key_imu_;
  Eigen::Matrix4d T_map_odom_ = Eigen::Matrix4d::Identity();
  // extrinsics
  Eigen::Matrix4d T_base_imu_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_lidar_imu_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_imu_lidar_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_base_lidar_ = Eigen::Matrix4d::Identity();
  // params
  float key_frame_distance_ = 2.0;
  double gravity_magnitude_ = 9.8;
  bool use_gnss_ = true;
  bool use_loop_closure_ = false;
  bool use_imu_pre_integration_ = false;
  int optimize_step_with_key_frame = 100;
  int optimize_step_with_loop = 10;
  Eigen::VectorXd lidar_odom_noise_;
  Eigen::VectorXd loop_closure_noise_;
  Eigen::VectorXd gnss_noise_;
  //
  bool has_new_key_frame_ = false;
  bool has_new_optimized_ = false;
  int new_loop_cnt_ = 0;
  int new_key_frame_cnt_ = 0;
};

}  // namespace loosely_lio_mapping
