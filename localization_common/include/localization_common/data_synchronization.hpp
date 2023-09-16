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
#include "localization_common/sensor_data/gnss_data.hpp"
#include "localization_common/sensor_data/imu_data.hpp"
#include "localization_common/sensor_data/twist_data.hpp"

namespace localization_common
{

bool sync_gnss_data(
  std::deque<GnssData> & unsynced_data, std::deque<GnssData> & synced_data, double sync_time);
bool sync_imu_data2(
  std::deque<ImuData2> & unsynced_data, std::deque<ImuData2> & synced_data, double sync_time);
bool sync_twist_data(
  std::deque<TwistData> & unsynced_data, std::deque<TwistData> & synced_data,
  double sync_time);

}  // namespace localization_common
