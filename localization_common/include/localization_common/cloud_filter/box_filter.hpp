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

#include <pcl/filters/crop_box.h>
#include <yaml-cpp/yaml.h>
#include <vector>

#include "localization_common/cloud_filter/cloud_filter_interface.hpp"

namespace localization_common
{
class BoxFilter : public CloudFilterInterface
{
  using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

public:
  explicit BoxFilter(YAML::Node node);

  PointCloudPtr apply(const PointCloudPtr & input) override;

  void set_size(std::vector<float> size);
  void set_origin(std::vector<float> origin);
  std::vector<float> get_edge();

private:
  void calculate_edge();

private:
  pcl::CropBox<pcl::PointXYZ> pcl_box_filter_;

  std::vector<float> origin_;
  std::vector<float> size_;
  std::vector<float> edge_;
};
}  // namespace localization_common
