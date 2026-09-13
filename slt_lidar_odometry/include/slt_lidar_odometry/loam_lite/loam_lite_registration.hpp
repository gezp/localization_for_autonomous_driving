// Copyright 2026 Gezp (https://github.com/gezp).
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
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "slt_common/tic_toc.hpp"
#include "slt_lidar_odometry/loam_lite/loam_lite_feature.hpp"

namespace slt_lidar_odometry
{

// scan to map registration: nearest neighbors + covariance fit
// (edge: line by largest eigen vector, surf: plane by smallest eigen vector),
// then ceres optimization with point-to-line / point-to-plane factors.
// reference: ALOAM/FLOAM/LIO-SAM
class LoamLiteRegistration
{
  using PointType = pcl::PointXYZ;
  using PointCloudType = pcl::PointCloud<pcl::PointXYZ>;
  using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

public:
  struct EdgeCorrespondence
  {
    Eigen::Vector3d current_point;
    Eigen::Vector3d line_point;
    Eigen::Vector3d line_direction;
  };
  struct SurfCorrespondence
  {
    Eigen::Vector3d current_point;
    Eigen::Vector3d plane_point;
    Eigen::Vector3d plane_normal;
  };

  LoamLiteRegistration() = default;
  ~LoamLiteRegistration() = default;
  explicit LoamLiteRegistration(const YAML::Node & config);
  // target is feature (in map frame, e.g. local map)
  bool set_target(const LoamLiteFeature & target);
  bool match(const LoamLiteFeature & input, const Eigen::Matrix4d & initial_pose);
  Eigen::Matrix4d get_final_pose();

private:
  std::vector<EdgeCorrespondence> find_all_edge_correspondence(
    const PointCloudType & point_cloud, const Eigen::Matrix4d & initial_pose);
  std::vector<SurfCorrespondence> find_all_surf_correspondence(
    const PointCloudType & point_cloud, const Eigen::Matrix4d & initial_pose);
  bool fit_line(
    const PointCloudType & point_cloud, const std::vector<int> & indices, Eigen::Vector3d & p0,
    Eigen::Vector3d & d);
  bool fit_plane(
    const PointCloudType & point_cloud, const std::vector<int> & indices, Eigen::Vector3d & p0,
    Eigen::Vector3d & n);

private:
  // correspondence search config
  int num_nearby_{5};
  double sqr_nearby_distance_{1.0};
  // covariance fit check
  double edge_eigen_ratio_{3.0};
  double max_line_distance_{0.3};
  double max_plane_distance_{0.2};
  // optimization config
  int num_optimization_{2};
  bool use_analytic_derivatives_{true};
  double ceres_loss_{0.1};
  int max_num_iterations_{4};
  int min_num_residual_blocks_{10};
  bool minimizer_progress_to_stdout_{false};
  bool debug_{false};
  // data
  PointCloudPtr edge_map_;
  PointCloudPtr surf_map_;
  pcl::KdTreeFLANN<PointType>::Ptr edge_kd_tree_;
  pcl::KdTreeFLANN<PointType>::Ptr surf_kd_tree_;
  // ceres parameter (qx,qy,qz,qw,tx,ty,tz)
  double ceres_parameter_[7];
  // debug
  slt_common::AdvancedTicToc elapsed_time_statistics_;
};

}  // namespace slt_lidar_odometry
