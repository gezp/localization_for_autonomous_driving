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

#include <Eigen/Geometry>

#include "localization_common/sensor_data/velocity_data.hpp"
#include "localization_common/sensor_data/pose_data.hpp"

namespace localization_common
{

Eigen::Quaternionf get_quaternion(const Eigen::Matrix4f & pose);
Eigen::Vector3f get_translation(const Eigen::Matrix4f & pose);
VelocityData get_velocity_data(const PoseData & pose_data);
VelocityData transform_velocity_data(
  const VelocityData & data, const Eigen::Matrix4f & transform_matrix);

}  // namespace localization_common
