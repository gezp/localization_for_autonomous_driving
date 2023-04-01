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

#include "localization_common/cloud_filter/no_filter.hpp"

namespace localization_common
{
NoFilter::NoFilter() {}

bool NoFilter::filter(const PointXYZCloudPtr & input_cloud, PointXYZCloudPtr & filtered_cloud)
{
  filtered_cloud.reset(new PointXYZCloud(*input_cloud));
  return true;
}
}  // namespace localization_common
