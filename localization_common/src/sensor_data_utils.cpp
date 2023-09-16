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

#include "localization_common/sensor_data_utils.hpp"

namespace localization_common
{

TwistData transform_twist(const TwistData & twist_a, const Eigen::Matrix4d & T_ab)
{
  TwistData twist_b;
  Eigen::Matrix3d R_ab = T_ab.block<3, 3>(0, 0);
  Eigen::Vector3d t_ab = T_ab.block<3, 1>(0, 3);
  Eigen::Vector3d v_ab = twist_a.linear_velocity + twist_a.angular_velocity.cross(t_ab);
  twist_b.angular_velocity = R_ab.inverse() * twist_a.angular_velocity;
  twist_b.linear_velocity = R_ab.inverse() * v_ab;
  return twist_b;
}

OdomData transform_odom(const OdomData & odom_a, const Eigen::Matrix4d & T_ab)
{
  OdomData odom_b;
  odom_b.time = odom_a.time;
  odom_b.pose = odom_a.pose * T_ab;
  Eigen::Matrix3d R_ab = T_ab.block<3, 3>(0, 0);
  Eigen::Vector3d t_ab = T_ab.block<3, 1>(0, 3);
  Eigen::Vector3d v_ab = odom_a.linear_velocity + odom_a.angular_velocity.cross(t_ab);
  odom_b.angular_velocity = R_ab.inverse() * odom_a.angular_velocity;
  odom_b.linear_velocity = R_ab.inverse() * v_ab;
  return odom_b;
}

Eigen::Vector3d interpolate_xyz(
  const Eigen::Vector3d & data1, const Eigen::Vector3d & data2, double coeff)
{
  assert(coeff >= 0 && coeff <= 1);
  return data1 * (1 - coeff) + data2 * coeff;
}

ImuData interpolate_imu(const ImuData & data1, const ImuData & data2, double time)
{
  assert(time >= data1.time && time <= data2.time);
  double coeff = (time - data1.time) / (data2.time - data1.time);
  ImuData output;
  output.time = time;
  output.linear_acceleration =
    interpolate_xyz(data1.linear_acceleration, data2.linear_acceleration, coeff);
  output.angular_velocity = interpolate_xyz(data1.angular_velocity, data2.angular_velocity, coeff);
  return output;
}

}  // namespace localization_common
