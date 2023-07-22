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

#include "localization_common/cloud_filter/voxel_filter.hpp"

namespace localization_common
{

VoxelFilter::VoxelFilter(const YAML::Node & node)
{
  float leaf_size_x = node["leaf_size"][0].as<float>();
  float leaf_size_y = node["leaf_size"][1].as<float>();
  float leaf_size_z = node["leaf_size"][2].as<float>();

  voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
}

VoxelFilter::VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z)
{
  voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
}

VoxelFilter::PointCloudPtr VoxelFilter::apply(const VoxelFilter::PointCloudPtr & input)
{
  PointCloudPtr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  voxel_filter_.setInputCloud(input);
  voxel_filter_.filter(*output_cloud);
  return output_cloud;
}

void VoxelFilter::print_info()
{
  std::cout << "[voxel_filter] "
            << "leaf size:" << voxel_filter_.getLeafSize().transpose() << std::endl;
}

}  // namespace localization_common
