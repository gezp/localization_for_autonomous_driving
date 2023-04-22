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

namespace kf_based_localization
{

struct NavState
{
  double time;
  // in world frame
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Matrix3d ori = Eigen::Matrix3d::Identity();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity;
  // in body frame
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d accl_bias = Eigen::Vector3d::Zero();
};

}  // namespace kf_based_localization
