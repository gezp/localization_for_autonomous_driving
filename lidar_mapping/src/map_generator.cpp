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

#include "lidar_mapping/map_generator.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <filesystem>
#include <fstream>

#include "localization_common/cloud_filter/voxel_filter.hpp"

namespace lidar_mapping
{
MapGenerator::MapGenerator()
{
  cloud_filter_factory_ = std::make_shared<localization_common::CloudFilterFactory>();
}

bool MapGenerator::init_config(const std::string & config_path, const std::string & data_path)
{
  YAML::Node config_node = YAML::LoadFile(config_path);
  // init data path
  data_path_ = data_path;
  key_frames_path_ = data_path_ + "/key_frames";
  map_path_ = data_path_ + "/map";
  // init filter
  display_filter_ = cloud_filter_factory_->create(config_node["display_filter"]);
  global_map_filter_ = cloud_filter_factory_->create(config_node["global_map_filter"]);
  return true;
}

localization_common::PointXYZCloudPtr MapGenerator::joint_cloud_map(
  const std::deque<localization_common::KeyFrame> & key_frames)
{
  localization_common::PointXYZCloudPtr map_cloud(new localization_common::PointXYZCloud());
  localization_common::PointXYZCloudPtr cloud(new localization_common::PointXYZCloud());
  std::string file_path = "";
  for (size_t i = 0; i < key_frames.size(); ++i) {
    file_path = key_frames_path_ + "/key_frame_" + std::to_string(key_frames.at(i).index) + ".pcd";
    pcl::io::loadPCDFile(file_path, *cloud);
    pcl::transformPointCloud(*cloud, *cloud, key_frames.at(i).pose);
    *map_cloud += *cloud;
  }
  return map_cloud;
}

localization_common::PointXYZCloudPtr MapGenerator::get_global_map(
  const std::deque<localization_common::KeyFrame> & optimized_key_frames, bool use_display_filter)
{
  auto global_map = joint_cloud_map(optimized_key_frames);
  if (use_display_filter) {
    display_filter_->filter(global_map, global_map);
  }
  return global_map;
}

bool MapGenerator::save_map(const std::deque<localization_common::KeyFrame> & optimized_key_frames)
{
  if (optimized_key_frames.size() == 0) {
    return false;
  }
  std::filesystem::remove_all(map_path_);
  if (!std::filesystem::create_directory(map_path_)) {
    return false;
  }
  std::cout << "start to SaveMap, key frames size:" << optimized_key_frames.size() << std::endl;
  // global map
  auto global_map = joint_cloud_map(optimized_key_frames);
  // save global map
  std::string map_file_path = map_path_ + "/map.pcd";
  pcl::io::savePCDFileBinary(map_file_path, *global_map);
  // filter global map
  if (global_map->points.size() > 1000000) {
    global_map_filter_->filter(global_map, global_map);
  }
  // save filtered global map
  std::string filtered_map_file_path = map_path_ + "/filtered_map.pcd";
  pcl::io::savePCDFileBinary(filtered_map_file_path, *global_map);
  //
  std::cout << "map dir:" << map_path_ << std::endl;
  return true;
}

}  // namespace lidar_mapping
