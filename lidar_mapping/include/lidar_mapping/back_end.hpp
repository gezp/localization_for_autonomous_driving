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
#include <vector>
#include <fstream>
#include <string>

#include "localization_common/sensor_data/lidar_data.hpp"
#include "localization_common/sensor_data/key_frame.hpp"
#include "localization_common/sensor_data/loop_pose.hpp"
#include "localization_common/sensor_data/odom_data.hpp"
#include "localization_common/cloud_filter/cloud_filter_factory.hpp"
#include "localization_common/lidar_key_frame_manager.hpp"
#include "lidar_mapping/graph_optimizer/g2o_graph_optimizer.hpp"

namespace lidar_mapping
{
class BackEnd
{
public:
  BackEnd();
  bool init_config(const std::string & config_path, const std::string & data_path);
  bool update(
    const localization_common::LidarData<pcl::PointXYZ> & lidar_data,
    const localization_common::OdomData & lidar_odom,
    const localization_common::OdomData & gnss_odom);
  bool insert_loop_pose(const localization_common::LoopPose & loop_pose);
  bool optimize(bool force = true);
  bool has_new_key_frame();
  bool has_new_optimized();
  const std::vector<localization_common::LidarFrame> & get_key_frames();
  Eigen::Matrix4d get_lidar_odom_to_map();
  pcl::PointCloud<pcl::PointXYZ>::Ptr get_global_map();
  bool save_map();

private:
  bool init_graph_optimizer(const YAML::Node & config_node);
  bool add_node_and_edge();
  bool check_new_key_frame(const localization_common::OdomData & lidar_odom);

private:
  std::shared_ptr<localization_common::LidarKeyFrameManager> key_frame_manager_;
  std::shared_ptr<localization_common::CloudFilterInterface> display_filter_;
  std::shared_ptr<localization_common::CloudFilterInterface> global_map_filter_;
  std::shared_ptr<localization_common::CloudFilterFactory> cloud_filter_factory_;
  // data
  Eigen::Matrix4d current_gnss_pose_;
  Eigen::Matrix4d current_lidar_pose_;
  Eigen::Matrix4d last_lidar_pose_;
  Eigen::Matrix4d pose_to_optimize_ = Eigen::Matrix4d::Identity();
  // optimizer
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
  float key_frame_distance_ = 2.0;
  int new_gnss_cnt_ = 0;
  int new_loop_cnt_ = 0;
  int new_key_frame_cnt_ = 0;
  bool has_new_key_frame_ = false;
  bool has_new_optimized_ = false;
};
}  // namespace lidar_mapping
