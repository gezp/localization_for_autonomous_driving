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

#include <memory>
#include <vector>

#include "slt_common/sensor_data/lidar_data.hpp"
#include "slt_lidar_odometry/loam_advanced/loam_advanced_feature.hpp"

namespace slt_lidar_odometry
{

// simple loam-like feature extraction: per-ring curvature, divided into sectors,
// pick edge points by curvature quota with neighbor suppression, rest as surf.
// reference: slam_in_autonomous_driving ch7 loam-like/feature_extraction.cc
class LoamAdvancedFeatureExtraction
{
  using PointType = pcl::PointXYZ;
  using PointCloudType = pcl::PointCloud<pcl::PointXYZ>;
  using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

public:
  LoamAdvancedFeatureExtraction() = default;
  ~LoamAdvancedFeatureExtraction() = default;
  explicit LoamAdvancedFeatureExtraction(const YAML::Node & config);
  using LidarCloudPtr = pcl::PointCloud<slt_common::PointXYZIRT>::Ptr;
  bool extract(const LidarCloudPtr & point_cloud, LoamAdvancedFeature & feature);

private:
  double get_point_curvature(const PointCloudType & ring_point_cloud, int idx);
  void mark_neighbor_points(
    const PointCloudType & ring_point_cloud, std::vector<bool> & mark, int idx);

private:
  // config
  int num_rings_{64};
  int num_sectors_{6};
  int min_point_size_in_ring_{131};
  // for get_point_curvature
  int curvature_padding_{5};
  // curvature threshold for edge points
  double edge_curvature_threshold_{0.1};
  // for mark_neighbor_points
  int neighbor_padding_{5};
  double neighbor_distance_threshold_{0.05};
  // max num edge points in each sector of each ring
  int max_num_edge_{20};
  bool debug_{false};
};

}  // namespace slt_lidar_odometry
