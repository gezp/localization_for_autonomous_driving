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

#include <ceres/ceres.h>
#include <Eigen/Dense>

namespace localization_common
{

struct LoamEdgeFactor
{
  LoamEdgeFactor(Eigen::Vector3d current_p_i, Eigen::Vector3d last_p_j, Eigen::Vector3d last_p_l)
  : current_p_i_(current_p_i), last_p_j_(last_p_j), last_p_l_(last_p_l)
  {
  }

  template<typename T>
  bool operator()(const T * q, const T * t, T * residual) const
  {
    Eigen::Map<const Eigen::Quaternion<T>> q_last_current(q);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_last_current(t);

    Eigen::Matrix<T, 3, 1> p = q_last_current.toRotationMatrix() * current_p_i_ + t_last_current;

    Eigen::Map<Eigen::Matrix<T, 3, 1>> d(residual);
    d = (p - last_p_j_).cross(p - last_p_l_) / (last_p_j_ - last_p_l_).norm();
    return true;
  }

  static ceres::CostFunction * create(
    const Eigen::Vector3d & current_p_i, const Eigen::Vector3d & last_p_j,
    const Eigen::Vector3d & last_p_l)
  {
    return new ceres::AutoDiffCostFunction<LoamEdgeFactor, 3, 4, 3>(
      new LoamEdgeFactor(current_p_i, last_p_j, last_p_l));
  }
  Eigen::Vector3d current_p_i_;
  Eigen::Vector3d last_p_j_;
  Eigen::Vector3d last_p_l_;
};

struct LoamPlanarFactor
{
  LoamPlanarFactor(
    Eigen::Vector3d current_p_i, Eigen::Vector3d last_p_j, Eigen::Vector3d last_p_l,
    Eigen::Vector3d last_p_m)
  : current_p_i_(current_p_i), last_p_j_(last_p_j), last_p_l_(last_p_l), last_p_m_(last_p_m)
  {
    last_n_jlm_ = (last_p_j_ - last_p_l_).cross(last_p_j_ - last_p_m_);
    last_n_jlm_.normalize();
  }

  template<typename T>
  bool operator()(const T * q, const T * t, T * residual) const
  {
    Eigen::Map<const Eigen::Quaternion<T>> q_last_current(q);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_last_current(t);
    Eigen::Matrix<T, 3, 1> p = q_last_current.toRotationMatrix() * current_p_i_ + t_last_current;

    residual[0] = (p - last_p_j_).dot(last_n_jlm_);
    return true;
  }

  static ceres::CostFunction * create(
    const Eigen::Vector3d current_p_i, const Eigen::Vector3d last_p_j,
    const Eigen::Vector3d last_p_l, const Eigen::Vector3d last_p_m)
  {
    return new ceres::AutoDiffCostFunction<LoamPlanarFactor, 1, 4, 3>(
      new LoamPlanarFactor(current_p_i, last_p_j, last_p_l, last_p_m));
  }
  Eigen::Vector3d current_p_i_;
  Eigen::Vector3d last_p_j_;
  Eigen::Vector3d last_p_l_;
  Eigen::Vector3d last_p_m_;
  Eigen::Vector3d last_n_jlm_;
};

}  // namespace localization_common
