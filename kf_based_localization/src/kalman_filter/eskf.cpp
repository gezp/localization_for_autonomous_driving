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

#include <cmath>
#include <iostream>
#include <sophus/so3.hpp>

#include "kf_based_localization/kalman_filter/eskf.hpp"

namespace kf_based_localization
{

Eskf::Eskf(const YAML::Node & node)
{
  // prior state covariance:
  covariance_.prior.pos = node["covariance"]["prior"]["pos"].as<double>();
  covariance_.prior.vel = node["covariance"]["prior"]["vel"].as<double>();
  covariance_.prior.ori = node["covariance"]["prior"]["ori"].as<double>();
  covariance_.prior.gyro_bias = node["covariance"]["prior"]["gyro_bias"].as<double>();
  covariance_.prior.accel_bias = node["covariance"]["prior"]["accel_bias"].as<double>();
  // process noise:
  covariance_.process.gyro = node["covariance"]["process"]["gyro"].as<double>();
  covariance_.process.accel = node["covariance"]["process"]["accel"].as<double>();
  covariance_.process.gyro_bias = node["covariance"]["process"]["gyro_bias"].as<double>();
  covariance_.process.accel_bias = node["covariance"]["process"]["accel_bias"].as<double>();
  // prompt:
  std::cout << "ESKF params:" << std::endl
            << "\tprior cov. pos: " << covariance_.prior.pos << std::endl
            << "\tprior cov. vel: " << covariance_.prior.vel << std::endl
            << "\tprior cov. ori: " << covariance_.prior.ori << std::endl
            << "\tprior cov. gyro_bias: " << covariance_.prior.gyro_bias << std::endl
            << "\tprior cov. accel_bias: " << covariance_.prior.accel_bias << std::endl
            << "\tprocess noise gyro.: " << covariance_.process.gyro << std::endl
            << "\tprocess noise accel.: " << covariance_.process.accel << std::endl
            << std::endl;
  // prior state & covariance:
  X_.setZero();
  reset_covariance();
  // process noise:
  Q_.setZero();
  Q_.block<3, 3>(kIndexNoiseAccel, kIndexNoiseAccel) =
    covariance_.process.accel * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(kIndexNoiseGyro, kIndexNoiseGyro) =
    covariance_.process.gyro * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(kIndexNoiseBiasAccel, kIndexNoiseBiasAccel) =
    covariance_.process.accel_bias * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(kIndexNoiseBiasGyro, kIndexNoiseBiasGyro) =
    covariance_.process.gyro_bias * Eigen::Matrix3d::Identity();
  // process equation
  F_.setZero();
  F_.block<3, 3>(kIndexErrorPos, kIndexErrorVel) = Eigen::Matrix3d::Identity();
  F_.block<3, 3>(kIndexErrorOri, kIndexErrorGyro) = -Eigen::Matrix3d::Identity();
  B_.setZero();
  B_.block<3, 3>(kIndexErrorOri, kIndexNoiseGyro) = Eigen::Matrix3d::Identity();
  B_.block<3, 3>(kIndexErrorAccel, kIndexNoiseBiasAccel) = Eigen::Matrix3d::Identity();
  B_.block<3, 3>(kIndexErrorGyro, kIndexNoiseBiasGyro) = Eigen::Matrix3d::Identity();
  // measurement equation
  HPose_.setZero();
  HPose_.block<3, 3>(0, kIndexErrorPos) = Eigen::Matrix3d::Identity();
  HPose_.block<3, 3>(3, kIndexErrorOri) = Eigen::Matrix3d::Identity();
  // imu integration
  imu_integration_ = std::make_shared<ImuIntegration>();
}

void Eskf::init_state(const NavState & nav_state, const localization_common::IMUData & imu_data)
{
  time_ = nav_state.time;
  pos_ = nav_state.pos;
  ori_ = nav_state.ori;
  vel_ = nav_state.vel;
  gravity_ = nav_state.gravity;
  accl_bias_ = nav_state.accl_bias;
  gyro_bias_ = nav_state.gyro_bias;
  // init imu integration
  imu_integration_->init(nav_state, imu_data);
}

bool Eskf::predict(const localization_common::IMUData & imu_data)
{
  // check
  if (imu_data.time < time_) {
    return false;
  }
  double dt = imu_data.time - time_;
  time_ = imu_data.time;
  // imu integration:
  imu_integration_->update(imu_data);
  auto state = imu_integration_->get_state();
  pos_ = state.pos;
  ori_ = state.ori;
  vel_ = state.vel;
  // update process equation:
  Eigen::Matrix3d R_wb = ori_;
  auto w_b = imu_data.angular_velocity;
  auto a_b = imu_data.linear_acceleration;
  F_.block<3, 3>(kIndexErrorVel, kIndexErrorOri) = -R_wb * Sophus::SO3d::hat(a_b);
  F_.block<3, 3>(kIndexErrorVel, kIndexErrorAccel) = -R_wb;
  B_.block<3, 3>(kIndexErrorVel, kIndexNoiseAccel) = R_wb;
  F_.block<3, 3>(kIndexErrorOri, kIndexErrorOri) = -Sophus::SO3d::hat(w_b);
  // get discretized process equations
  Eigen::Matrix<double, kDimState, kDimState> F =
    Eigen::Matrix<double, kDimState, kDimState>::Identity() + F_ * dt;
  Eigen::Matrix<double, kDimState, kDimProcessNoise> B =
    Eigen::Matrix<double, kDimState, kDimProcessNoise>::Zero();
  B.block<6, 6>(3, 0) = B_.block<6, 6>(3, 0) * dt;
  B.block<6, 6>(9, 6) = B_.block<6, 6>(9, 6) * sqrt(dt);
  // perform Kalman prediction
  X_ = F * X_;
  P_ = F * P_ * F.transpose() + B * Q_ * B.transpose();
  return true;
}

bool Eskf::observe_pose(const Eigen::Matrix4d & pose, const Eigen::Matrix<double, 6, 1> & noise)
{
  // create measurement Y
  Eigen::Matrix<double, kDimMeasurementPose, 1> Y;
  Y.block<3, 1>(0, 0) = pos_ - pose.block<3, 1>(0, 3);
  auto dR = pose.block<3, 3>(0, 0).transpose() * ori_;
  Y.block<3, 1>(3, 0) = Sophus::SO3d::vee(dR - Eigen::Matrix3d::Identity());
  // measurement equation H, V
  auto H = HPose_;
  Eigen::Matrix<double, kDimMeasurementPose, kDimMeasurementPose> V = noise.asDiagonal();
  // get kalman gain
  auto K = P_ * H.transpose() * (H * P_ * H.transpose() + V).inverse();
  // perform Kalman correct
  X_ = X_ + K * (Y - H * X_);
  P_ = (Eigen::Matrix<double, kDimState, kDimState>::Identity() - K * H) * P_;
  // eliminate error
  eliminate_error();
  // reset error state
  X_ = Eigen::Matrix<double, kDimState, 1>::Zero();
  return true;
}

double Eskf::get_time() {return time_;}

NavState Eskf::get_nav_state()
{
  NavState nav_state;
  nav_state.time = time_;
  nav_state.pos = pos_;
  nav_state.ori = ori_;
  nav_state.vel = vel_;
  nav_state.gravity = gravity_;
  nav_state.gyro_bias = gyro_bias_;
  nav_state.accl_bias = accl_bias_;
  return nav_state;
}

void Eskf::eliminate_error(void)
{
  // update pos, vel, ori
  pos_ -= X_.block<3, 1>(kIndexErrorPos, 0);
  vel_ -= X_.block<3, 1>(kIndexErrorVel, 0);
  auto dR = Eigen::Matrix3d::Identity() - Sophus::SO3d::hat(X_.block<3, 1>(kIndexErrorOri, 0));
  auto R = ori_ * dR;
  ori_ = Eigen::Quaterniond(R).normalized().toRotationMatrix();
  // update bias
  if (is_cov_stable(kIndexErrorGyro)) {
    gyro_bias_ -= X_.block<3, 1>(kIndexErrorGyro, 0);
  }
  if (is_cov_stable(kIndexErrorAccel)) {
    accl_bias_ -= X_.block<3, 1>(kIndexErrorAccel, 0);
  }
  // update imu integration
  NavState state;
  state.pos = pos_;
  state.ori = ori_;
  state.vel = vel_;
  state.accl_bias = accl_bias_;
  state.gyro_bias = gyro_bias_;
  state.gravity = gravity_;
  imu_integration_->set_state(state);
}

bool Eskf::is_cov_stable(int index_offset, double thresh)
{
  for (int i = 0; i < 3; ++i) {
    if (P_(index_offset + i, index_offset + i) > thresh) {
      return false;
    }
  }
  return true;
}

void Eskf::reset_covariance(void)
{
  P_.setZero();
  P_.block<3, 3>(kIndexErrorPos, kIndexErrorPos) =
    covariance_.prior.pos * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(kIndexErrorVel, kIndexErrorVel) =
    covariance_.prior.vel * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(kIndexErrorOri, kIndexErrorOri) =
    covariance_.prior.ori * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(kIndexErrorGyro, kIndexErrorGyro) =
    covariance_.prior.gyro_bias * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(kIndexErrorAccel, kIndexErrorAccel) =
    covariance_.prior.accel_bias * Eigen::Matrix3d::Identity();
}

}  // namespace kf_based_localization
