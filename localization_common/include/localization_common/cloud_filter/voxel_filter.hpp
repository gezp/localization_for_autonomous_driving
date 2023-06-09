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

#include <pcl/filters/voxel_grid.h>
#include <yaml-cpp/yaml.h>

#include "localization_common/cloud_filter/cloud_filter_interface.hpp"

namespace localization_common
{

class VoxelFilter : public CloudFilterInterface
{
public:
  explicit VoxelFilter(const YAML::Node & node);
  VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);

  bool filter(const PointXYZCloudPtr & input_cloud, PointXYZCloudPtr & filtered_cloud) override;

private:
  bool set_param(float leaf_size_x, float leaf_size_y, float leaf_size_z);

private:
  pcl::VoxelGrid<PointXYZ> voxel_filter_;
};

}  // namespace localization_common
