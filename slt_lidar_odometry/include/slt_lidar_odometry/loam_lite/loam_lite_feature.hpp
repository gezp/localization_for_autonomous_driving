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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>

namespace slt_lidar_odometry
{

struct LoamLiteFeature
{
  using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
  PointCloudPtr edge;
  PointCloudPtr surf;
};

// transform feature point clouds (edge/surf) by pose
LoamLiteFeature transform_feature(
  const LoamLiteFeature & input, const Eigen::Matrix4d & pose);

// get feature point cloud for rviz display (edge_rgb for edge points, surf_rgb for surf points)
pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_feature_point_cloud(
  const LoamLiteFeature & feature, const std::vector<int> & edge_rgb,
  const std::vector<int> & surf_rgb);

}  // namespace slt_lidar_odometry
