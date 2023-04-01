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

#include <pcl/common/transforms.h>

#include <Eigen/Dense>

#include "localization_common/distortion_adjust.hpp"
#include "localization_common/sensor_data/cloud_data.hpp"
#include "localization_common/sensor_data/velocity_data.hpp"

namespace localization_common
{
class DistortionAdjust
{
public:
  void set_motion_info(float scan_period, VelocityData velocity_data);
  bool adjust_cloud(PointXYZCloudPtr & input_cloud, PointXYZCloudPtr & output_cloud);

private:
  inline Eigen::Matrix3f update_matrix(float real_time);

private:
  float scan_period_;
  Eigen::Vector3f velocity_;
  Eigen::Vector3f angular_rate_;
};
}  // namespace localization_common
