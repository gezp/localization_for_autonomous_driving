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

#include "loosely_lio_mapping/graph_optimizer/imu_pre_integration.hpp"

#include <sophus/so3.hpp>

namespace loosely_lio_mapping
{

ImuPreIntegration::ImuPreIntegration(ImuPreIntegrationNoise noise)
{
  // init bias
  state_.b_a_i = Eigen::Vector3d::Zero();
  state_.b_g_i = Eigen::Vector3d::Zero();
  // process noise
  Q_.setZero();
  Q_.block<3, 3>(INDEX_M_ACC_PREV, INDEX_M_ACC_PREV) =
    Q_.block<3, 3>(INDEX_M_ACC_CURR, INDEX_M_ACC_CURR) = noise.accel * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(INDEX_M_GYR_PREV, INDEX_M_GYR_PREV) =
    Q_.block<3, 3>(INDEX_M_GYR_CURR, INDEX_M_GYR_CURR) = noise.gyro * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(INDEX_R_ACC_PREV, INDEX_R_ACC_PREV) =
    noise.accel_bias * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(INDEX_R_GYR_PREV, INDEX_R_GYR_PREV) =
    noise.gyro_bias * Eigen::Matrix3d::Identity();
  // process equation, state propagation
  F_.setZero();
  F_.block<3, 3>(INDEX_ALPHA, INDEX_BETA) = Eigen::Matrix3d::Identity();
  F_.block<3, 3>(INDEX_THETA, INDEX_B_G) = -Eigen::Matrix3d::Identity();

  // process equation, noise input
  B_.setZero();
  B_.block<3, 3>(INDEX_THETA, INDEX_M_GYR_PREV) = B_.block<3, 3>(INDEX_THETA, INDEX_M_GYR_CURR) =
    0.50 * Eigen::Matrix3d::Identity();
  B_.block<3, 3>(INDEX_B_A, INDEX_R_ACC_PREV) = B_.block<3, 3>(INDEX_B_G, INDEX_R_GYR_PREV) =
    Eigen::Matrix3d::Identity();
}

void ImuPreIntegration::set_bias(Eigen::Vector3d b_a_i, Eigen::Vector3d b_g_i)
{
  state_.b_a_i = b_a_i;
  state_.b_g_i = b_g_i;
}

bool ImuPreIntegration::integrate(const localization_common::IMUData & imu_data)
{
  if (imu_data_buff_.empty()) {
    imu_data_buff_.push_back(imu_data);
    reset();
    return true;
  }

  if (imu_data_buff_.front().time > imu_data.time) {
    return false;
  }

  // set buffer:
  imu_data_buff_.push_back(imu_data);

  // update state mean, covariance and Jacobian:
  update_state();

  // move forward:
  imu_data_buff_.pop_front();
  return true;
}

void ImuPreIntegration::update_state(void)
{
  // get measurements
  Eigen::Vector3d w0 = imu_data_buff_.at(0).angular_velocity - state_.b_g_i;
  Eigen::Vector3d w1 = imu_data_buff_.at(1).angular_velocity - state_.b_g_i;
  Eigen::Vector3d a0 = imu_data_buff_.at(0).linear_acceleration - state_.b_a_i;
  Eigen::Vector3d a1 = imu_data_buff_.at(1).linear_acceleration - state_.b_a_i;
  double dt = imu_data_buff_.at(1).time - imu_data_buff_.at(0).time;
  state_.dt = imu_data_buff_.at(1).time - time_;
  // update
  Eigen::Vector3d w_mid = 0.5 * (w0 + w1);
  Eigen::Matrix3d new_theta_ij = state_.theta_ij * Sophus::SO3d::exp(w_mid * dt).matrix();
  Eigen::Vector3d a_mid = 0.5 * (state_.theta_ij * a0 + new_theta_ij * a1);
  Eigen::Vector3d new_beta_ij = state_.beta_ij + a_mid * dt;
  Eigen::Vector3d new_alpha_ij = state_.alpha_ij + state_.beta_ij * dt + 0.5 * a_mid * dt * dt;
  // update covariance
  // intermediate results
  auto dt2 = dt * dt;
  Eigen::Matrix3d prev_R = state_.theta_ij;
  Eigen::Matrix3d curr_R = new_theta_ij;
  Eigen::Matrix3d prev_R_a_hat = prev_R * Sophus::SO3d::hat(a0);
  Eigen::Matrix3d curr_R_a_hat = curr_R * Sophus::SO3d::hat(a1);
  Eigen::Matrix3d dR_inv = Eigen::Matrix3d::Identity() - Sophus::SO3d::hat(w_mid) * dt;
  // set up F: F = I + F_ * T
  // F12 & F14 & F15:
  F_.block<3, 3>(INDEX_ALPHA, INDEX_THETA) = -0.25 * (prev_R_a_hat + curr_R_a_hat * dR_inv) * dt;
  F_.block<3, 3>(INDEX_ALPHA, INDEX_B_A) = -0.25 * (prev_R + curr_R) * dt;
  F_.block<3, 3>(INDEX_ALPHA, INDEX_B_G) = 0.25 * curr_R_a_hat * dt2;
  // F22:
  F_.block<3, 3>(INDEX_THETA, INDEX_THETA) = -Sophus::SO3d::hat(w_mid);
  // F32 & F34 & F35
  F_.block<3, 3>(INDEX_BETA, INDEX_THETA) = -0.5 * (prev_R_a_hat + curr_R_a_hat * dR_inv);
  F_.block<3, 3>(INDEX_BETA, INDEX_B_A) = -0.5 * (prev_R + curr_R);
  F_.block<3, 3>(INDEX_BETA, INDEX_B_G) = 0.5 * curr_R_a_hat * dt;
  Eigen::Matrix<double, DIM_STATE, DIM_STATE> F;
  F = Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Identity() + F_ * dt;
  // set up B: B + B_ * T
  // B11 & B12 & B13 & B14:
  B_.block<3, 3>(INDEX_M_ACC_PREV, INDEX_M_ACC_PREV) = 0.25 * prev_R * dt;
  B_.block<3, 3>(INDEX_M_ACC_PREV, INDEX_M_GYR_PREV) = -0.125 * curr_R_a_hat * dt2;
  B_.block<3, 3>(INDEX_M_ACC_PREV, INDEX_M_ACC_CURR) = 0.25 * curr_R * dt;
  B_.block<3, 3>(INDEX_M_ACC_PREV, INDEX_M_GYR_CURR) = -0.125 * curr_R_a_hat * dt2;
  // B31 & B32 & B33 & B34:
  B_.block<3, 3>(INDEX_M_ACC_CURR, INDEX_M_ACC_PREV) = 0.5 * prev_R;
  B_.block<3, 3>(INDEX_M_ACC_CURR, INDEX_M_GYR_PREV) = -0.25 * curr_R_a_hat * dt;
  B_.block<3, 3>(INDEX_M_ACC_CURR, INDEX_M_ACC_CURR) = 0.5 * curr_R;
  B_.block<3, 3>(INDEX_M_ACC_CURR, INDEX_M_GYR_CURR) = -0.25 * curr_R_a_hat * dt;
  Eigen::Matrix<double, DIM_STATE, DIM_NOISE> B = B_ * dt;
  // update P_:
  state_.P = F * state_.P * F.transpose() + B * Q_ * B.transpose();
  // update Jacobian:
  state_.J = F * state_.J;
  // update
  state_.alpha_ij = new_alpha_ij;
  state_.theta_ij = new_theta_ij;
  state_.beta_ij = new_beta_ij;
}

const ImuPreIntegrationState & ImuPreIntegration::get_state() {return state_;}

bool ImuPreIntegration::reset()
{
  if (imu_data_buff_.empty()) {
    return false;
  }
  time_ = imu_data_buff_.front().time;
  state_.alpha_ij = Eigen::Vector3d::Zero();
  state_.theta_ij = Eigen::Matrix3d::Identity();
  state_.beta_ij = Eigen::Vector3d::Zero();
  state_.P.setZero();
  state_.J.setIdentity();
  return true;
}

}  // namespace loosely_lio_mapping
