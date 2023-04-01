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

#include <Eigen/Dense>

#include "localization_common/sensor_data/velocity_data.hpp"

namespace localization_common
{

class PoseData
{
public:
  double time = 0.0;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

  struct
  {
    Eigen::Vector3f v = Eigen::Vector3f::Zero();
    Eigen::Vector3f w = Eigen::Vector3f::Zero();
  } vel;

public:
  Eigen::Quaternionf get_quaternion();

  void get_velocity_data(VelocityData & velocity_data) const;
};

}  // namespace localization_common
