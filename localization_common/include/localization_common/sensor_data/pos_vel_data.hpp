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

#include <Eigen/Dense>

namespace localization_common
{

class PosVelData
{
public:
  double time = 0.0;

  Eigen::Vector3f pos = Eigen::Vector3f::Zero();
  Eigen::Vector3f vel = Eigen::Vector3f::Zero();
};

}  // namespace localization_common
