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

#include "localization_common/cloud_filter/cloud_filter_factory.hpp"

#include "localization_common/cloud_filter/box_filter.hpp"
#include "localization_common/cloud_filter/no_filter.hpp"
#include "localization_common/cloud_filter/voxel_filter.hpp"

namespace localization_common
{
std::shared_ptr<CloudFilterInterface> CloudFilterFactory::create(const YAML::Node & config)
{
  auto filter_mothod = config["filter_method"].as<std::string>();
  std::shared_ptr<CloudFilterInterface> filter_ptr = nullptr;
  if (filter_mothod == "voxel_filter") {
    filter_ptr = std::make_shared<VoxelFilter>(config["voxel_filter"]);
  } else if (filter_mothod == "box_filter") {
    filter_ptr = std::make_shared<VoxelFilter>(config["box_filter"]);
  } else if (filter_mothod == "no_filter") {
    filter_ptr = std::make_shared<NoFilter>();
  } else {
    std::cerr << "Point cloud filter method " << filter_mothod << " NOT FOUND!";
  }
  return filter_ptr;
}

}  // namespace localization_common
