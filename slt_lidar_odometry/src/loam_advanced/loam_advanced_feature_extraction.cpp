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

#include "slt_lidar_odometry/loam_advanced/loam_advanced_feature_extraction.hpp"

// reference: gaoxiang12/slam_in_autonomous_driving src/ch7/loam-like/feature_extraction.cc

namespace slt_lidar_odometry
{

LoamAdvancedFeatureExtraction::LoamAdvancedFeatureExtraction(const YAML::Node & config)
{
  num_rings_ = config["num_rings"].as<int>();
  num_sectors_ = config["num_sectors"].as<int>();
  //
  curvature_padding_ = config["curvature_padding"].as<int>();
  edge_curvature_threshold_ = config["edge_curvature_threshold"].as<double>();
  //
  neighbor_padding_ = config["neighbor_padding"].as<int>();
  neighbor_distance_threshold_ = config["neighbor_distance_threshold"].as<double>();
  //
  max_num_edge_ = config["max_num_edge"].as<int>();
  debug_ = config["debug"].as<bool>();
}

bool LoamAdvancedFeatureExtraction::extract(
  const LidarCloudPtr & point_cloud, LoamAdvancedFeature & feature)
{
  feature.edge.reset(new PointCloudType);
  feature.surf.reset(new PointCloudType);
  std::vector<PointCloudType> ring_point_clouds(num_rings_);
  // divide point cloud by ring id
  for (auto & p : point_cloud->points) {
    if (p.ring >= num_rings_ || std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
      continue;
    }
    PointType point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    ring_point_clouds[p.ring].emplace_back(point);
  }
  // extract feature for each ring
  for (int i = 0; i < num_rings_; i++) {
    auto & ring_point_cloud = ring_point_clouds[i];
    int point_size_in_ring = ring_point_clouds[i].points.size();
    if (point_size_in_ring < min_point_size_in_ring_) {
      continue;
    }
    // curvature, index sorted by curvature, picked_state for each point in current ring
    std::vector<double> point_curvature(point_size_in_ring, -1);
    std::vector<int> sorted_point_idx(point_size_in_ring, -1);
    std::vector<bool> point_picked_state(point_size_in_ring, false);
    // calculate curvature, need add padding.
    for (int j = curvature_padding_; j < point_size_in_ring - curvature_padding_; j++) {
      sorted_point_idx[j] = j;
      point_curvature[j] = get_point_curvature(ring_point_cloud, j);
    }
    // divide in some sectors for more uniform feature
    int valid_point_size_in_ring = (point_size_in_ring - curvature_padding_ * 2);
    for (int j = 0; j < num_sectors_; j++) {
      int start_idx = curvature_padding_ + valid_point_size_in_ring * j / num_sectors_;
      int end_idx = curvature_padding_ + valid_point_size_in_ring * (j + 1) / num_sectors_ - 1;
      // sort sorted_point_idx by point_curvature
      auto cmp = [&](const int & a, const int & b) {
          return point_curvature[a] < point_curvature[b];
        };
      std::sort(sorted_point_idx.begin() + start_idx, sorted_point_idx.begin() + end_idx + 1, cmp);
      // edge points
      int largest_picked_num = 0;
      for (int k = end_idx; k >= start_idx; k--) {
        int idx = sorted_point_idx[k];
        if (!point_picked_state[idx] && point_curvature[idx] > edge_curvature_threshold_) {
          largest_picked_num++;
          point_picked_state[idx] = true;
          if (largest_picked_num <= max_num_edge_) {
            feature.edge->push_back(ring_point_cloud.points[idx]);
          } else {
            break;
          }
          mark_neighbor_points(ring_point_cloud, point_picked_state, idx);
        }
      }
      // surf points: all unpicked points
      for (int k = start_idx; k <= end_idx; k++) {
        int idx = sorted_point_idx[k];
        if (!point_picked_state[idx]) {
          feature.surf->push_back(ring_point_cloud.points[idx]);
        }
      }
    }
  }
  if (debug_) {
    std::cout << "edge number: " << feature.edge->points.size() << ", surf number: "
              << feature.surf->points.size() << std::endl;
  }
  return true;
}

void LoamAdvancedFeatureExtraction::mark_neighbor_points(
  const PointCloudType & ring_point_cloud, std::vector<bool> & mark, int idx)
{
  // mark neighbor point
  for (int k = 1; k <= neighbor_padding_; k++) {
    if (idx + k >= static_cast<int>(ring_point_cloud.points.size())) {
      break;
    }
    double diff_x = ring_point_cloud.points[idx + k].x - ring_point_cloud.points[idx + k - 1].x;
    double diff_y = ring_point_cloud.points[idx + k].y - ring_point_cloud.points[idx + k - 1].y;
    double diff_z = ring_point_cloud.points[idx + k].z - ring_point_cloud.points[idx + k - 1].z;
    // break if it's discontinuous
    if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > neighbor_distance_threshold_) {
      break;
    }
    mark[idx + k] = true;
  }
  for (int k = -1; k >= -neighbor_padding_; k--) {
    if (idx + k < 0) {
      break;
    }
    double diff_x = ring_point_cloud.points[idx + k].x - ring_point_cloud.points[idx + k + 1].x;
    double diff_y = ring_point_cloud.points[idx + k].y - ring_point_cloud.points[idx + k + 1].y;
    double diff_z = ring_point_cloud.points[idx + k].z - ring_point_cloud.points[idx + k + 1].z;
    if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > neighbor_distance_threshold_) {
      break;
    }
    mark[idx + k] = true;
  }
}

double LoamAdvancedFeatureExtraction::get_point_curvature(
  const PointCloudType & ring_point_cloud, int idx)
{
  double diff_x = 0;
  double diff_y = 0;
  double diff_z = 0;
  for (int i = -curvature_padding_; i <= curvature_padding_; i++) {
    diff_x += ring_point_cloud.points[idx + i].x - ring_point_cloud.points[idx].x;
    diff_y += ring_point_cloud.points[idx + i].y - ring_point_cloud.points[idx].y;
    diff_z += ring_point_cloud.points[idx + i].z - ring_point_cloud.points[idx].z;
  }
  return diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
}

}  // namespace slt_lidar_odometry
