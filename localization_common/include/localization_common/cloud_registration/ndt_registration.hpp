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

#include <pcl/registration/ndt.h>
#include <yaml-cpp/yaml.h>

#include "localization_common/cloud_registration/cloud_registration_interface.hpp"

namespace localization_common
{
class NDTRegistration : public CloudRegistrationInterface
{
  using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

public:
  explicit NDTRegistration(const YAML::Node & node);
  NDTRegistration(float res, float step_size, float trans_eps, int max_iter);

  bool set_target(const PointCloudPtr & target) override;
  bool match(const PointCloudPtr & input, const Eigen::Matrix4f & initial_pose) override;
  Eigen::Matrix4f get_final_pose() override;
  double get_fitness_score() override;
  void print_info() override;

private:
  bool set_param(float res, float step_size, float trans_eps, int max_iter);

private:
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_;
};
}  // namespace localization_common
