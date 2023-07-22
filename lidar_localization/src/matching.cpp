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

#include "lidar_localization/matching.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

namespace lidar_localization
{
Matching::Matching()
: global_map_(new localization_common::PointXYZCloud()),
  local_map_(new localization_common::PointXYZCloud()),
  current_scan_(new localization_common::PointXYZCloud())
{
  registration_factory_ = std::make_shared<localization_common::CloudRegistrationFactory>();
  cloud_filter_factory_ = std::make_shared<localization_common::CloudFilterFactory>();
}

bool Matching::init_config(const std::string & config_path, const std::string & data_path)
{
  YAML::Node config_node = YAML::LoadFile(config_path);
  // init_data_path
  data_path_ = data_path;
  map_path_ = data_path_ + "/map/filtered_map.pcd";
  scan_context_path_ = data_path_ + "/scan_context";
  // init scan context
  init_scan_context(config_node);
  // init registration
  registration_ = registration_factory_->create(config_node);
  // init filter
  box_filter_ =
    std::make_shared<localization_common::BoxFilter>(config_node["roi_filter"]["box_filter"]);
  current_scan_filter_ = cloud_filter_factory_->create(config_node["current_scan_filter"]);
  // b. local map filter -- downsample & ROI filtering for scan-map matching:
  local_map_filter_ = cloud_filter_factory_->create(config_node["local_map_filter"]);
  // c. global map filter -- downsample point cloud map for visualization:
  global_map_filter_ = cloud_filter_factory_->create(config_node["global_map_filter"]);
  init_global_map();
  reset_local_map(0.0, 0.0, 0.0);
  // print info
  std::cout << "local_map roi filter:" << std::endl;
  box_filter_->print_info();
  std::cout << "current_scan filter:" << std::endl;
  current_scan_filter_->print_info();
  std::cout << "visualization local_map filter:" << std::endl;
  local_map_filter_->print_info();
  std::cout << "visualization global_map filter:" << std::endl;
  global_map_filter_->print_info();
  return true;
}

bool Matching::init_scan_context(const YAML::Node & config_node)
{
  // get loop closure config:
  std::string loop_closure_method = config_node["loop_closure_method"].as<std::string>();
  if (loop_closure_method == "scan_context") {
    // create instance:
    scan_context_manager_ =
      std::make_shared<scan_context::ScanContextManager>(config_node[loop_closure_method]);
    // load pre-built index:
    scan_context_manager_->load(scan_context_path_);
    return true;
  }
  return false;
}

bool Matching::init_global_map()
{
  pcl::io::loadPCDFile(map_path_, *global_map_);
  std::cout << "Load global map, size:" << global_map_->points.size();
  // since scan-map matching is used, here apply the same filter to local map
  // & scan:
  global_map_ = local_map_filter_->apply(global_map_);
  std::cout << "Filtered global map, size:" << global_map_->points.size();
  has_new_global_map_ = true;
  return true;
}

bool Matching::reset_local_map(float x, float y, float z)
{
  std::vector<float> origin = {x, y, z};
  // use ROI filtering for local map segmentation:
  box_filter_->set_origin(origin);
  local_map_ = box_filter_->apply(global_map_);
  registration_->set_input_target(local_map_);
  has_new_local_map_ = true;
  std::vector<float> edge = box_filter_->get_edge();
  std::cout << "New local map:" << edge.at(0) << "," << edge.at(1) << "," << edge.at(2) << ","
            << edge.at(3) << "," << edge.at(4) << "," << edge.at(5) << std::endl
            << std::endl;
  return true;
}

bool Matching::update(
  const localization_common::CloudData & cloud_data, Eigen::Matrix4f & cloud_pose)
{
  static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
  static Eigen::Matrix4f last_pose = init_pose_;
  static Eigen::Matrix4f predict_pose = init_pose_;

  // remove invalid measurements:
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_data.cloud, *cloud_data.cloud, indices);

  // downsample:
  auto filtered_cloud = current_scan_filter_->apply(cloud_data.cloud);

  if (!has_inited_) {
    predict_pose = current_gnss_pose_;
  }
  // matching:
  localization_common::PointXYZCloudPtr result_cloud(new localization_common::PointXYZCloud());
  registration_->match(filtered_cloud, predict_pose, result_cloud, cloud_pose);
  pcl::transformPointCloud(*cloud_data.cloud, *current_scan_, cloud_pose);
  // update predicted pose:
  step_pose = last_pose.inverse() * cloud_pose;
  predict_pose = cloud_pose * step_pose;
  last_pose = cloud_pose;
  // 匹配之后判断是否需要更新局部地图
  std::vector<float> edge = box_filter_->get_edge();
  for (int i = 0; i < 3; i++) {
    if (
      fabs(cloud_pose(i, 3) - edge.at(2 * i)) > 50.0 &&
      fabs(cloud_pose(i, 3) - edge.at(2 * i + 1)) > 50.0)
    {
      continue;
    }
    reset_local_map(cloud_pose(0, 3), cloud_pose(1, 3), cloud_pose(2, 3));
    break;
  }
  return true;
}

bool Matching::set_init_pose_by_gnss(const Eigen::Matrix4f & gnss_pose)
{
  static int gnss_cnt = 0;
  current_gnss_pose_ = gnss_pose;
  if (gnss_cnt == 0) {
    set_init_pose(gnss_pose);
  } else if (gnss_cnt > 3) {
    has_inited_ = true;
  }
  gnss_cnt++;
  return true;
}

bool Matching::set_init_pose_by_scan_context(const localization_common::CloudData & init_scan)
{
  // get init pose proposal using scan context match:
  Eigen::Matrix4f init_pose = Eigen::Matrix4f::Identity();
  if (!scan_context_manager_->detect_loop_closure(init_scan.cloud, init_pose)) {
    return false;
  }
  // set init pose:
  set_init_pose(init_pose);
  has_inited_ = true;
  return true;
}

bool Matching::set_init_pose(const Eigen::Matrix4f & init_pose)
{
  init_pose_ = init_pose;
  reset_local_map(init_pose(0, 3), init_pose(1, 3), init_pose(2, 3));
  return true;
}

Eigen::Matrix4f Matching::get_init_pose(void) {return init_pose_;}

localization_common::PointXYZCloudPtr Matching::get_global_map()
{
  return global_map_filter_->apply(global_map_);
}

localization_common::PointXYZCloudPtr Matching::get_local_map() {return local_map_;}

localization_common::PointXYZCloudPtr Matching::get_current_scan() {return current_scan_;}

bool Matching::has_inited() {return has_inited_;}

bool Matching::has_new_global_map() {return has_new_global_map_;}

bool Matching::has_new_local_map() {return has_new_local_map_;}

}  // namespace lidar_localization
