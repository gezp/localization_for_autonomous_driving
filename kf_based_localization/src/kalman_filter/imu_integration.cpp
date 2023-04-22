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

#include "kf_based_localization/kalman_filter/imu_integration.hpp"

#include <sophus/so3.hpp>

namespace kf_based_localization
{

ImuIntegration::ImuIntegration() {}

bool ImuIntegration::init(const NavState & state, const localization_common::IMUData & imu_data)
{
  state_ = state;
  imu_data_buff_.clear();
  imu_data_buff_.push_back(imu_data);
  return true;
}

bool ImuIntegration::update(const localization_common::IMUData & imu_data)
{
  // check
  if (imu_data.time < state_.time) {
    return false;
  }
  // imu integration:
  imu_data_buff_.push_back(imu_data);
  // get deltas:
  double dt = imu_data_buff_.at(1).time - imu_data_buff_.at(0).time;
  // phi
  auto w0 = imu_data_buff_.at(0).angular_velocity - state_.gyro_bias;
  auto w1 = imu_data_buff_.at(1).angular_velocity - state_.gyro_bias;
  auto phi = 0.5 * (w0 + w1) * dt;
  // ori
  auto new_ori = state_.ori * Sophus::SO3d::exp(phi).matrix();
  // vel
  auto a0 = imu_data_buff_.at(0).linear_acceleration - state_.accl_bias;
  auto a1 = imu_data_buff_.at(1).linear_acceleration - state_.accl_bias;
  auto new_vel = state_.vel + (0.5 * (state_.ori * a0 + new_ori * a1) + state_.gravity) * dt;
  // pos
  auto new_pos = state_.pos + 0.5 * (state_.vel + new_vel) * dt;
  // update
  state_.time = imu_data.time;
  state_.pos = new_pos;
  state_.ori = new_ori;
  state_.vel = new_vel;
  imu_data_buff_.pop_front();
  return true;
}

void ImuIntegration::set_state(const NavState & state) {state_ = state;}

NavState ImuIntegration::get_state() {return state_;}

}  // namespace kf_based_localization
