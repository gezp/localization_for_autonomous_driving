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
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"

namespace localization_common
{

geometry_msgs::msg::TransformStamped to_transform_stamped_msg(
  Eigen::Vector3d t, Eigen::Quaterniond q, double time);

geometry_msgs::msg::TransformStamped to_transform_stamped_msg(Eigen::Matrix4f pose, double time);

bool lookup_in_tf_buffer(
  std::shared_ptr<tf2_ros::Buffer> buffer, std::string base_frame_id, std::string child_frame_id,
  Eigen::Matrix4f & transform_matrix);
}  // namespace localization_common
