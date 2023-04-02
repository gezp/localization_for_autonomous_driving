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

#include "lidar_odometry/front_end.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>

#include <filesystem>
#include <fstream>

namespace lidar_odometry
{
FrontEnd::FrontEnd()
: local_map_(new localization_common::PointXYZCloud()),
  result_cloud_(new localization_common::PointXYZCloud())
{
  registration_factory_ = std::make_shared<localization_common::RegistrationFactory>();
  cloud_filter_factory_ = std::make_shared<localization_common::CloudFilterFactory>();
}

bool FrontEnd::init_config(const std::string & config_path, const std::string & data_path)
{
  YAML::Node config_node = YAML::LoadFile(config_path);
  // init_data_path
  data_path_ = data_path;
  // key_frame dir
  std::string key_frame_path = data_path_ + "/key_frames";
  if (std::filesystem::is_directory(key_frame_path)) {
    std::filesystem::remove_all(key_frame_path);
  }
  std::filesystem::create_directory(key_frame_path);
  std::cout << "Key Frames Output Path: " << key_frame_path << std::endl;
  //
  key_frame_distance_ = config_node["key_frame_distance"].as<float>();
  local_frame_num_ = config_node["local_frame_num"].as<int>();
  // init registration and filter
  registration_ = registration_factory_->create(config_node);
  local_map_filter_ = cloud_filter_factory_->create(config_node["local_map_filter"]);
  current_scan_filter_ = cloud_filter_factory_->create(config_node["current_scan_filter"]);
  display_filter_ = cloud_filter_factory_->create(config_node["display_filter"]);
  return true;
}

bool FrontEnd::update(
  const localization_common::CloudData & cloud_data, Eigen::Matrix4f & cloud_pose)
{
  current_frame_.cloud_data.time = cloud_data.time;
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_data.cloud, *current_frame_.cloud_data.cloud, indices);

  localization_common::PointXYZCloudPtr filtered_cloud(new localization_common::PointXYZCloud());
  current_scan_filter_->filter(current_frame_.cloud_data.cloud, filtered_cloud);

  static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
  static Eigen::Matrix4f last_pose = init_pose_;
  static Eigen::Matrix4f predict_pose = init_pose_;
  static Eigen::Matrix4f last_key_frame_pose = init_pose_;

  // 局部地图容器中没有关键帧，代表是第一帧数据
  // 此时把当前帧数据作为第一个关键帧，并更新局部地图容器和全局地图容器
  if (local_map_frames_.size() == 0) {
    current_frame_.pose = init_pose_;
    update_with_new_frame(current_frame_);
    cloud_pose = current_frame_.pose;
    return true;
  }

  // 不是第一帧，就正常匹配
  registration_->match(filtered_cloud, predict_pose, result_cloud_, current_frame_.pose);
  cloud_pose = current_frame_.pose;

  // 更新相邻两帧的相对运动
  step_pose = last_pose.inverse() * current_frame_.pose;
  predict_pose = current_frame_.pose * step_pose;
  last_pose = current_frame_.pose;

  // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
  if (
    fabs(last_key_frame_pose(0, 3) - current_frame_.pose(0, 3)) +
    fabs(last_key_frame_pose(1, 3) - current_frame_.pose(1, 3)) +
    fabs(last_key_frame_pose(2, 3) - current_frame_.pose(2, 3)) >
    key_frame_distance_)
  {
    update_with_new_frame(current_frame_);
    last_key_frame_pose = current_frame_.pose;
  }

  return true;
}

bool FrontEnd::set_init_pose(const Eigen::Matrix4f & init_pose)
{
  init_pose_ = init_pose;
  return true;
}

bool FrontEnd::update_with_new_frame(const Frame & new_key_frame)
{
  // 把关键帧点云存储到硬盘里，节省内存
  std::string file_path =
    data_path_ + "/key_frames/key_frame_" + std::to_string(global_map_frames_.size()) + ".pcd";
  pcl::io::savePCDFileBinary(file_path, *new_key_frame.cloud_data.cloud);

  Frame key_frame = new_key_frame;
  // 这一步的目的是为了把关键帧的点云保存下来
  // 由于用的是共享指针，所以直接复制只是复制了一个指针而已
  // 此时无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云
  key_frame.cloud_data.cloud.reset(
    new localization_common::PointXYZCloud(*new_key_frame.cloud_data.cloud));
  localization_common::PointXYZCloudPtr transformed_cloud(new localization_common::PointXYZCloud());

  // 更新局部地图
  local_map_frames_.push_back(key_frame);
  while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_)) {
    local_map_frames_.pop_front();
  }
  local_map_.reset(new localization_common::PointXYZCloud());
  for (size_t i = 0; i < local_map_frames_.size(); ++i) {
    pcl::transformPointCloud(
      *local_map_frames_.at(i).cloud_data.cloud, *transformed_cloud, local_map_frames_.at(i).pose);
    *local_map_ += *transformed_cloud;
  }
  has_new_local_map_ = true;

  // 更新ndt匹配的目标点云
  // 关键帧数量还比较少的时候不滤波，因为点云本来就不多，太稀疏影响匹配效果
  if (local_map_frames_.size() < 10) {
    registration_->set_input_target(local_map_);
  } else {
    localization_common::PointXYZCloudPtr filtered_local_map(
      new localization_common::PointXYZCloud());
    local_map_filter_->filter(local_map_, filtered_local_map);
    registration_->set_input_target(filtered_local_map);
  }

  // 保存所有关键帧信息在容器里
  // 存储之前，点云要先释放，因为已经存到了硬盘里，不释放也达不到节省内存的目的
  key_frame.cloud_data.cloud.reset(new localization_common::PointXYZCloud());
  global_map_frames_.push_back(key_frame);

  return true;
}

bool FrontEnd::get_new_local_map(localization_common::PointXYZCloudPtr & local_map)
{
  if (has_new_local_map_) {
    display_filter_->filter(local_map_, local_map);
    return true;
  }
  return false;
}

bool FrontEnd::get_new_global_map(localization_common::PointXYZCloudPtr & global_map)
{
  global_map.reset(new localization_common::PointXYZCloud());

  std::string key_frame_path = "";
  localization_common::PointXYZCloudPtr key_frame_cloud(new localization_common::PointXYZCloud());
  localization_common::PointXYZCloudPtr transformed_cloud(new localization_common::PointXYZCloud());

  for (size_t i = 0; i < global_map_frames_.size(); ++i) {
    key_frame_path = data_path_ + "/key_frames/key_frame_" + std::to_string(i) + ".pcd";
    pcl::io::loadPCDFile(key_frame_path, *key_frame_cloud);

    pcl::transformPointCloud(*key_frame_cloud, *transformed_cloud, global_map_frames_.at(i).pose);
    *global_map += *transformed_cloud;
  }
  display_filter_->filter(global_map, global_map);
  return true;
}

bool FrontEnd::get_current_scan(localization_common::PointXYZCloudPtr & current_scan)
{
  display_filter_->filter(result_cloud_, current_scan);
  return true;
}
}  // namespace lidar_odometry
