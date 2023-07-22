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

#include "localization_common/cloud_filter/box_filter.hpp"

#include <iostream>
#include <vector>

namespace localization_common
{
BoxFilter::BoxFilter(YAML::Node node)
{
  size_.resize(6);
  edge_.resize(6);
  origin_.resize(3);

  for (size_t i = 0; i < size_.size(); i++) {
    size_.at(i) = node["box_filter_size"][i].as<float>();
  }
  set_size(size_);
}

BoxFilter::PointCloudPtr BoxFilter::apply(const BoxFilter::PointCloudPtr & input)
{
  PointCloudPtr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl_box_filter_.setMin(Eigen::Vector4f(edge_.at(0), edge_.at(2), edge_.at(4), 1.0e-6));
  pcl_box_filter_.setMax(Eigen::Vector4f(edge_.at(1), edge_.at(3), edge_.at(5), 1.0e6));
  pcl_box_filter_.setInputCloud(input);
  pcl_box_filter_.filter(*output_cloud);
  return output_cloud;
}

void BoxFilter::print_info()
{
  std::cout << "[box_filter] "
            << "min_x: " << size_.at(0) << ", "
            << "max_x: " << size_.at(1) << ", "
            << "min_y: " << size_.at(2) << ", "
            << "max_y: " << size_.at(3) << ", "
            << "min_z: " << size_.at(4) << ", "
            << "max_z: " << size_.at(5) << std::endl;
}

void BoxFilter::set_size(std::vector<float> size)
{
  size_ = size;
  calculate_edge();
}

void BoxFilter::set_origin(std::vector<float> origin)
{
  origin_ = origin;
  calculate_edge();
}

void BoxFilter::calculate_edge()
{
  for (size_t i = 0; i < origin_.size(); ++i) {
    edge_.at(2 * i) = size_.at(2 * i) + origin_.at(i);
    edge_.at(2 * i + 1) = size_.at(2 * i + 1) + origin_.at(i);
  }
}

std::vector<float> BoxFilter::get_edge() {return edge_;}
}  // namespace localization_common
