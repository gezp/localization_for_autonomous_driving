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

#include "slt_lidar_odometry/loam_advanced/loam_advanced_feature.hpp"

#include <pcl/common/transforms.h>

namespace slt_lidar_odometry
{

LoamAdvancedFeature transform_feature(
  const LoamAdvancedFeature & input, const Eigen::Matrix4d & pose)
{
  LoamAdvancedFeature output;
  output.edge.reset(new pcl::PointCloud<pcl::PointXYZ>);
  output.surf.reset(new pcl::PointCloud<pcl::PointXYZ>);
  if (input.edge->size() > 0) {
    pcl::transformPointCloud(*input.edge, *output.edge, pose);
  }
  if (input.surf->size() > 0) {
    pcl::transformPointCloud(*input.surf, *output.surf, pose);
  }
  return output;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_feature_point_cloud(
  const LoamAdvancedFeature & feature, const std::vector<int> & edge_rgb,
  const std::vector<int> & surf_rgb)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (auto & p : feature.edge->points) {
    pcl::PointXYZRGB point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.r = edge_rgb[0];
    point.g = edge_rgb[1];
    point.b = edge_rgb[2];
    point_cloud->push_back(point);
  }
  for (auto & p : feature.surf->points) {
    pcl::PointXYZRGB point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.r = surf_rgb[0];
    point.g = surf_rgb[1];
    point.b = surf_rgb[2];
    point_cloud->push_back(point);
  }
  return point_cloud;
}

}  // namespace slt_lidar_odometry
