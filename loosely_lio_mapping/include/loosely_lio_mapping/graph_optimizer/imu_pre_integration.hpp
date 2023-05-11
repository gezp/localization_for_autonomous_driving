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

#include <deque>

#include "localization_common/sensor_data/imu_data.hpp"
#include "localization_common/sensor_data/imu_nav_state.hpp"

namespace loosely_lio_mapping
{

struct ImuPreIntegrationNoise
{
  double prior;
  double gyro;
  double accel;
  double gyro_bias;
  double accel_bias;
};

struct ImuPreIntegrationState
{
  static const int DIM_STATE = 15;
  // time delta:
  double dt;
  // relative translation
  Eigen::Vector3d alpha_ij;
  // relative orientation
  Eigen::Matrix3d theta_ij;
  // relative velocity
  Eigen::Vector3d beta_ij;
  // accel bias
  Eigen::Vector3d b_a_i;
  // gyro bias
  Eigen::Vector3d b_g_i;
  // information matrix
  Eigen::Matrix<double, DIM_STATE, DIM_STATE> P;
  // Jacobian for update caused by bias
  Eigen::Matrix<double, DIM_STATE, DIM_STATE> J;
};

class ImuPreIntegration
{
public:
  explicit ImuPreIntegration(ImuPreIntegrationNoise noise);
  void set_bias(Eigen::Vector3d b_a_i, Eigen::Vector3d b_g_i);
  bool integrate(const localization_common::IMUData & imu_data);
  const ImuPreIntegrationState & get_state();
  double get_time() {return time_;}
  bool reset();

private:
  void update_state();

private:
  static const int DIM_STATE = 15;
  static const int DIM_NOISE = 18;

  static const int INDEX_ALPHA = 0;
  static const int INDEX_THETA = 3;
  static const int INDEX_BETA = 6;
  static const int INDEX_B_A = 9;
  static const int INDEX_B_G = 12;

  static const int INDEX_M_ACC_PREV = 0;
  static const int INDEX_M_GYR_PREV = 3;
  static const int INDEX_M_ACC_CURR = 6;
  static const int INDEX_M_GYR_CURR = 9;
  static const int INDEX_R_ACC_PREV = 12;
  static const int INDEX_R_GYR_PREV = 15;

  // hyper-params:
  double prior_noise_;
  double gyro_noise_;
  double accel_noise_;
  double gyro_bias_noise_;
  double accel_bias_noise_;

  // data buff:
  std::deque<localization_common::IMUData> imu_data_buff_;
  // pre-integration state:
  double time_;
  ImuPreIntegrationState state_;
  // process noise:
  Eigen::Matrix<double, DIM_NOISE, DIM_NOISE> Q_;
  // process equation:
  Eigen::Matrix<double, DIM_STATE, DIM_STATE> F_;
  Eigen::Matrix<double, DIM_STATE, DIM_NOISE> B_;
};

}  // namespace loosely_lio_mapping
