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

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include "localization_common/sensor_data/cloud_data.hpp"

namespace localization_common
{
class CloudRegistrationInterface
{
public:
  virtual ~CloudRegistrationInterface() = default;

  virtual bool set_input_target(const PointXYZCloudPtr & input_target) = 0;
  virtual bool match(
    const PointXYZCloudPtr & input_source, const Eigen::Matrix4f & predict_pose,
    PointXYZCloudPtr & result_cloud, Eigen::Matrix4f & result_pose) = 0;
  virtual float get_fitness_score() = 0;
};
}  // namespace localization_common
