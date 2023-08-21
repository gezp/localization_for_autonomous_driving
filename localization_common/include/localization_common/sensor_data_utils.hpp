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
#include "localization_common/sensor_data/imu_data.hpp"
#include "localization_common/sensor_data/odom_data.hpp"

namespace localization_common
{

Eigen::Quaternionf get_quaternion(const Eigen::Matrix4f & pose);
Eigen::Vector3f get_translation(const Eigen::Matrix4f & pose);
VelocityData get_velocity_data(const OdomData & odom);
VelocityData transform_velocity_data(
  const VelocityData & data, const Eigen::Matrix4f & transform_matrix);

// Odom wa -> Odom wb by Tab
OdomData transform_odom(const OdomData & odom_a, const Eigen::Matrix4d & T_ab);

Eigen::Vector3d interpolate_xyz(
  const Eigen::Vector3d & data1, const Eigen::Vector3d & data2, double coeff);

ImuData interpolate_imu(const ImuData & data1, const ImuData & data2, double time);

}  // namespace localization_common
