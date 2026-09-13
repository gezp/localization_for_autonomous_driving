// Copyright 2026 Gezp (https://github.com/gezp).
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
#include <sophus/common.hpp>
#include <sophus/so3.hpp>

namespace slt_lidar_odometry
{

class SO3Manifold : public ceres::Manifold
{
public:
  virtual bool Plus(const double * x, const double * delta, double * x_plus_delta) const
  {
    Eigen::Map<const Eigen::Quaterniond> q(x);
    Eigen::Map<const Eigen::Vector3d> delta_so3(delta);
    Eigen::Map<Eigen::Quaterniond> q_plus_delta(x_plus_delta);
    Eigen::Quaterniond delta_q = Sophus::SO3d::exp(delta_so3).unit_quaternion();
    q_plus_delta = (delta_q * q).normalized();
    return true;
  }
  virtual bool PlusJacobian(const double * /*x*/, double * jacobian) const
  {
    Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian);
    (j.topRows(3)).setIdentity();
    (j.bottomRows(1)).setZero();
    return true;
  }
  virtual int AmbientSize() const {return 4;}
  virtual int TangentSize() const {return 3;}

  virtual bool Minus(const double * y, const double * x, double * y_minus_x) const
  {
    Eigen::Map<const Eigen::Quaterniond> q_y(y);
    Eigen::Map<const Eigen::Quaterniond> q_x(x);
    Eigen::Map<Eigen::Vector3d> delta(y_minus_x);
    delta = (Sophus::SO3d(q_y) * Sophus::SO3d(q_x).inverse()).log();
    return true;
  }
  virtual bool MinusJacobian(const double * /*x*/, double * jacobian) const
  {
    Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> J(jacobian);
    J.setIdentity();
    return true;
  }
};

// point to line: line defined by point p_j and unit direction d (from covariance fit)
struct LoamAdvancedEdgeFactor
{
  LoamAdvancedEdgeFactor(Eigen::Vector3d current_p_i, Eigen::Vector3d p_j, Eigen::Vector3d d)
  : current_p_i_(current_p_i), p_j_(p_j), d_(d.normalized())
  {
  }

  template<typename T>
  bool operator()(const T * q, const T * t, T * residual) const
  {
    Eigen::Map<const Eigen::Quaternion<T>> q_map_current(q);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_map_current(t);

    Eigen::Matrix<T, 3, 1> p = q_map_current.toRotationMatrix() * current_p_i_ + t_map_current;

    Eigen::Matrix<T, 3, 1> diff = p - p_j_;
    Eigen::Matrix<T, 3, 1> d_T = d_.template cast<T>();
    // residual = diff - d * (d . diff), scalar ops to keep Jet types happy
    T d_dot_diff = d_T[0] * diff[0] + d_T[1] * diff[1] + d_T[2] * diff[2];
    residual[0] = diff[0] - d_T[0] * d_dot_diff;
    residual[1] = diff[1] - d_T[1] * d_dot_diff;
    residual[2] = diff[2] - d_T[2] * d_dot_diff;
    return true;
  }

  static ceres::CostFunction * create(
    const Eigen::Vector3d & current_p_i, const Eigen::Vector3d & p_j, const Eigen::Vector3d & d)
  {
    return new ceres::AutoDiffCostFunction<LoamAdvancedEdgeFactor, 3, 4, 3>(
      new LoamAdvancedEdgeFactor(current_p_i, p_j, d));
  }
  Eigen::Vector3d current_p_i_;
  Eigen::Vector3d p_j_;
  Eigen::Vector3d d_;
};

// point to plane: plane defined by point p_j and unit normal n (from covariance fit)
struct LoamAdvancedSurfFactor
{
  LoamAdvancedSurfFactor(Eigen::Vector3d current_p_i, Eigen::Vector3d p_j, Eigen::Vector3d n)
  : current_p_i_(current_p_i), p_j_(p_j), n_(n.normalized())
  {
  }

  template<typename T>
  bool operator()(const T * q, const T * t, T * residual) const
  {
    Eigen::Map<const Eigen::Quaternion<T>> q_map_current(q);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_map_current(t);

    Eigen::Matrix<T, 3, 1> p = q_map_current.toRotationMatrix() * current_p_i_ + t_map_current;

    Eigen::Matrix<T, 3, 1> diff = p - p_j_;
    Eigen::Matrix<T, 3, 1> n_T = n_.template cast<T>();
    // scalar ops to keep Jet types happy
    residual[0] = n_T[0] * diff[0] + n_T[1] * diff[1] + n_T[2] * diff[2];
    return true;
  }

  static ceres::CostFunction * create(
    const Eigen::Vector3d & current_p_i, const Eigen::Vector3d & p_j, const Eigen::Vector3d & n)
  {
    return new ceres::AutoDiffCostFunction<LoamAdvancedSurfFactor, 1, 4, 3>(
      new LoamAdvancedSurfFactor(current_p_i, p_j, n));
  }
  Eigen::Vector3d current_p_i_;
  Eigen::Vector3d p_j_;
  Eigen::Vector3d n_;
};

// point to line: line defined by point p_j and unit direction d (from covariance fit)
// residual r = (I - d d^T) (R * p_i + t - p_j)
// must pair with SO3Manifold (left perturbation q' = delta_q * q, identity PlusJacobian):
// jacobian: dr/ddelta = -(I - d d^T) [R * p_i]_x, dr/dt = (I - d d^T)
class LoamAdvancedEdgeAnalyticFactor : public ceres::SizedCostFunction<3, 4, 3>
{
public:
  LoamAdvancedEdgeAnalyticFactor(
    Eigen::Vector3d current_p_i, Eigen::Vector3d p_j,
    Eigen::Vector3d d)
  : current_p_i_(current_p_i), p_j_(p_j)
  {
    d_ = d.normalized();
    // projection matrix to the plane perpendicular to line direction
    P_ = Eigen::Matrix3d::Identity() - d_ * d_.transpose();
  }

  virtual bool Evaluate(
    double const * const * parameters, double * residuals, double ** jacobians) const
  {
    Eigen::Map<const Eigen::Quaterniond> q_map_current(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_map_current(parameters[1]);

    Eigen::Vector3d p = q_map_current * current_p_i_ + t_map_current;
    Eigen::Vector3d diff = p - p_j_;

    Eigen::Map<Eigen::Vector3d> r(residuals);
    r = P_ * diff;

    if (jacobians) {
      if (jacobians[0]) {
        // dp/ddelta = -[R * p_i]_x (left perturbation, paired with SO3Manifold)
        Eigen::Matrix3d dp_by_so3 = -Sophus::SO3d::hat(q_map_current * current_p_i_);
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> J_q(jacobians[0]);
        J_q.setZero();
        J_q.block<3, 3>(0, 0) = P_ * dp_by_so3;
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J_t(jacobians[1]);
        J_t = P_;
      }
    }
    return true;
  }
  Eigen::Vector3d current_p_i_;
  Eigen::Vector3d p_j_;
  Eigen::Vector3d d_;
  Eigen::Matrix3d P_;
};

// point to plane: plane defined by point p_j and unit normal n (from covariance fit)
// residual r = n^T (R * p_i + t - p_j)
// must pair with SO3Manifold (left perturbation q' = delta_q * q, identity PlusJacobian):
// jacobian: dr/ddelta = -n^T [R * p_i]_x, dr/dt = n^T
class LoamAdvancedSurfAnalyticFactor : public ceres::SizedCostFunction<1, 4, 3>
{
public:
  LoamAdvancedSurfAnalyticFactor(
    Eigen::Vector3d current_p_i, Eigen::Vector3d p_j, Eigen::Vector3d n)
  : current_p_i_(current_p_i), p_j_(p_j)
  {
    n_ = n.normalized();
  }

  virtual bool Evaluate(
    double const * const * parameters, double * residuals, double ** jacobians) const
  {
    Eigen::Map<const Eigen::Quaterniond> q_map_current(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_map_current(parameters[1]);

    Eigen::Vector3d p = q_map_current * current_p_i_ + t_map_current;
    Eigen::Vector3d diff = p - p_j_;

    residuals[0] = n_.dot(diff);

    if (jacobians) {
      Eigen::Matrix<double, 1, 3> dh_by_dp = n_.transpose();
      if (jacobians[0]) {
        // dp/ddelta = -[R * p_i]_x (left perturbation, paired with SO3Manifold)
        Eigen::Matrix3d dp_by_so3 = -Sophus::SO3d::hat(q_map_current * current_p_i_);
        Eigen::Map<Eigen::Matrix<double, 1, 4>> J_q(jacobians[0]);
        J_q.setZero();
        J_q.block<1, 3>(0, 0) = dh_by_dp * dp_by_so3;
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 1, 3>> J_t(jacobians[1]);
        J_t = dh_by_dp;
      }
    }
    return true;
  }
  Eigen::Vector3d current_p_i_;
  Eigen::Vector3d p_j_;
  Eigen::Vector3d n_;
};

}  // namespace slt_lidar_odometry
