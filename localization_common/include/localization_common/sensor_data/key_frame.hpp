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

namespace localization_common
{
class KeyFrame
{
public:
  double time = 0.0;
  unsigned int index = 0;
  // a. position & orientation:
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  // b. velocity:
  struct
  {
    Eigen::Vector3f v = Eigen::Vector3f::Zero();
    Eigen::Vector3f w = Eigen::Vector3f::Zero();
  } vel;
  // c. bias:
  struct
  {
    // c.1. accelerometer:
    Eigen::Vector3f accel = Eigen::Vector3f::Zero();
    // c.2. gyroscope:
    Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
  } bias;
};
}  // namespace localization_common
