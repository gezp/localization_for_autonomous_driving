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

#include "localization_common/tic_toc.hpp"

namespace localization_common
{

TicToc::TicToc()
{
  tic();
}

void TicToc::tic()
{
  start_ = std::chrono::steady_clock::now();
}

double TicToc::toc()
{
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start_;
  return elapsed_seconds.count() * 1000;
}

}  // namespace localization_common
