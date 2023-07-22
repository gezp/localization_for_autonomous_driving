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
  registration_factory_ = std::make_shared<localization_common::CloudRegistrationFactory>();
  cloud_filter_factory_ = std::make_shared<localization_common::CloudFilterFactory>();
}

bool FrontEnd::init_config(const std::string & config_path)
{
  YAML::Node config_node = YAML::LoadFile(config_path);
  //
  local_frame_num_ = config_node["local_frame_num"].as<int>();
  key_frame_distance_ = config_node["key_frame_distance"].as<float>();
  // init registration and filter
  registration_ = registration_factory_->create(config_node);
  local_map_filter_ = cloud_filter_factory_->create(config_node["local_map_filter"]);
  current_scan_filter_ = cloud_filter_factory_->create(config_node["current_scan_filter"]);
  display_filter_ = cloud_filter_factory_->create(config_node["display_filter"]);
  // print info
  std::cout << "cloud registration:" << std::endl;
  registration_->print_info();
  std::cout << "local_map filter:" << std::endl;
  local_map_filter_->print_info();
  std::cout << "current_scan filter:" << std::endl;
  current_scan_filter_->print_info();
  std::cout << "display filter:" << std::endl;
  display_filter_->print_info();
  return true;
}

bool FrontEnd::update(
  const localization_common::CloudData & cloud_data, Eigen::Matrix4f & cloud_pose)
{
  static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
  static Eigen::Matrix4f last_pose = init_pose_;
  static Eigen::Matrix4f predict_pose = init_pose_;
  static Eigen::Matrix4f last_key_frame_pose = init_pose_;

  // reset param
  has_new_local_map_ = false;

  current_frame_.cloud_data.time = cloud_data.time;
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_data.cloud, *current_frame_.cloud_data.cloud, indices);
  auto filtered_cloud = current_scan_filter_->apply(current_frame_.cloud_data.cloud);

  // 局部地图容器中没有关键帧，代表是第一帧数据
  // 此时把当前帧数据作为第一个关键帧，并更新局部地图容器和全局地图容器
  if (local_map_frames_.size() == 0) {
    current_frame_.pose = init_pose_;
    update_with_new_frame(current_frame_);
    cloud_pose = current_frame_.pose;
    return true;
  }

  // 不是第一帧，就正常匹配
  registration_->match(filtered_cloud, predict_pose);
  current_frame_.pose = registration_->get_final_pose();
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
  Frame key_frame = new_key_frame;
  // 这一步的目的是为了把关键帧的点云保存下来
  // 由于用的是共享指针，所以直接复制只是复制了一个指针而已
  // 此时无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云
  key_frame.cloud_data.cloud.reset(
    new localization_common::PointXYZCloud(*new_key_frame.cloud_data.cloud));

  // 更新局部地图
  local_map_frames_.push_back(key_frame);
  while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_)) {
    local_map_frames_.pop_front();
  }

  local_map_.reset(new localization_common::PointXYZCloud());
  localization_common::PointXYZCloudPtr transformed_cloud(new localization_common::PointXYZCloud());
  for (size_t i = 0; i < local_map_frames_.size(); ++i) {
    pcl::transformPointCloud(
      *local_map_frames_.at(i).cloud_data.cloud, *transformed_cloud, local_map_frames_.at(i).pose);
    *local_map_ += *transformed_cloud;
  }
  has_new_local_map_ = true;

  // 更新ndt匹配的目标点云
  // 关键帧数量还比较少的时候不滤波，因为点云本来就不多，太稀疏影响匹配效果
  if (local_map_frames_.size() < 10) {
    registration_->set_target(local_map_);
  } else {
    auto filtered_local_map = local_map_filter_->apply(local_map_);
    registration_->set_target(filtered_local_map);
  }

  return true;
}

bool FrontEnd::has_new_local_map() {return has_new_local_map_;}

localization_common::PointXYZCloudPtr FrontEnd::get_local_map()
{
  return display_filter_->apply(local_map_);
}

localization_common::PointXYZCloudPtr FrontEnd::get_current_scan()
{
  return display_filter_->apply(result_cloud_);
}

}  // namespace lidar_odometry
