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

#include "localization_common/kitti_preprocess_node.hpp"

#include "localization_common/sensor_data_utils.hpp"
#include "localization_common/msg_utils.hpp"

namespace localization_common
{
KittiPreprocessNode::KittiPreprocessNode(rclcpp::Node::SharedPtr node)
{
  node->declare_parameter("use_manual_map_origin", use_manual_map_origin_);
  node->declare_parameter("map_origin", map_origin_);
  node->get_parameter("use_manual_map_origin", use_manual_map_origin_);
  node->get_parameter("map_origin", map_origin_);
  if (map_origin_.size() != 3) {
    RCLCPP_FATAL(
      node->get_logger(), "map_origin's size must be 3 for (latitude, longitude, altitude)");
    return;
  }
  imu_frame_id_ = "imu_link";
  lidar_frame_id_ = "velo_link";
  base_frame_id_ = "base_link";
  // subscriber
  cloud_sub_ =
    std::make_shared<CloudSubscriber<pcl::PointXYZ>>(node, "/kitti/velo/pointcloud", 10000);
  nav_sat_fix_sub_ = std::make_shared<NavSatFixSubscriber>(node, "/kitti/oxts/gps/fix", 10000);
  twist_sub_ = std::make_shared<TwistSubscriber>(node, "/kitti/oxts/gps/vel", 10000);
  imu_sub_ = std::make_shared<ImuSubscriber>(node, "/kitti/oxts/imu", 10000);
  if (use_manual_map_origin_) {
    nav_sat_fix_sub_->set_map_origin(map_origin_[0], map_origin_[1], map_origin_[2]);
  }
  // publisher
  cloud_pub_ =
    std::make_shared<CloudPublisher<pcl::PointXYZ>>(node, "synced_cloud", base_frame_id_, 100);
  gnss_data_pub_ = std::make_shared<GnssPublisher>(node, "/kitti/gnss_data", 100);
  gnss_odom_pub_ =
    std::make_shared<OdometryPublisher>(node, "synced_gnss/pose", "map", base_frame_id_, 100);
  // extrinsics
  extrinsics_manager_ = std::make_shared<ExtrinsicsManager>(node);
  extrinsics_manager_->enable_tf_listener();
  // motion compensation for lidar measurement:
  distortion_adjust_ = std::make_shared<DistortionAdjust>();
  //
  run_thread_ = std::make_unique<std::thread>(
    [this]() {
      while (run_flag_) {
        if (!run()) {
          using namespace std::chrono_literals;
          std::this_thread::sleep_for(10ms);
        }
      }
    });
}

bool KittiPreprocessNode::run()
{
  // get calibrated extrinsics
  if (!is_valid_extrinsics_) {
    if (!extrinsics_manager_->lookup(imu_frame_id_, base_frame_id_, T_imu_base_)) {
      return false;
    }
    if (!extrinsics_manager_->lookup(base_frame_id_, lidar_frame_id_, T_base_lidar_)) {
      return false;
    }
    if (!extrinsics_manager_->lookup(imu_frame_id_, lidar_frame_id_, T_imu_lidar_)) {
      return false;
    }
    is_valid_extrinsics_ = true;
  }
  // read data
  read_data();
  bool valid_data = false;
  // process gnss data
  while (has_gnss_data()) {
    if (valid_gnss_data()) {
      // gnss odometry (position from GNSS, orientation from IMU)
      OdomData odom_imu;
      odom_imu.time = current_gnss_data_.time;
      odom_imu.pose.block<3, 1>(0, 3) = current_gnss_data_.antenna_position;
      odom_imu.pose.block<3, 3>(0, 0) = current_imu_data_.orientation.matrix();
      odom_imu.linear_velocity = current_twist_data_.linear_velocity;
      odom_imu.angular_velocity = current_twist_data_.angular_velocity;
      auto odom = transform_odom(odom_imu, T_imu_base_);
      // add into buffer
      gnss_odom_buffer_.push_back(odom);
      // publish gnss data and odometry
      gnss_data_pub_->publish(current_gnss_data_);
      gnss_odom_pub_->publish(odom);
      valid_data = true;
    }
  }
  // process lidar data
  while (!lidar_data_buffer_.empty()) {
    auto current_lidar_data = lidar_data_buffer_.front();
    // drop old lidar data
    if (!gnss_odom_buffer_.empty() && current_lidar_data.time < gnss_odom_buffer_.front().time) {
      lidar_data_buffer_.pop_front();
      continue;
    }
    // get sync gnss odom
    OdomData synced_odom;
    if (!get_synced_gnss(current_lidar_data.time, synced_odom)) {
      break;
    }
    auto odom_lidar = transform_odom(synced_odom, T_base_lidar_);
    TwistData twist_lidar;
    twist_lidar.time = odom_lidar.time;
    twist_lidar.linear_velocity = odom_lidar.linear_velocity;
    twist_lidar.angular_velocity = odom_lidar.angular_velocity;
    // motion compensation for lidar  point cloud
    distortion_adjust_->set_motion_info(0.1, twist_lidar);
    distortion_adjust_->adjust_cloud(
      current_lidar_data.point_cloud, current_lidar_data.point_cloud);
    // transform lidar point cloud
    Eigen::Matrix4d pose = T_base_lidar_.inverse();
    pcl::transformPointCloud(
      *current_lidar_data.point_cloud, *current_lidar_data.point_cloud, pose);
    // publish lidar point cloud
    cloud_pub_->publish(current_lidar_data.point_cloud, current_lidar_data.time);
    // update buffer
    lidar_data_buffer_.pop_front();
    valid_data = true;
  }
  return valid_data;
}

bool KittiPreprocessNode::read_data()
{
  cloud_sub_->parse_data(lidar_data_buffer_);
  nav_sat_fix_sub_->parse_data(gnss_data_buffer_);
  twist_sub_->parse_data(twist_data_buffer_);
  imu_sub_->parse_data(imu_data_buffer_);
  return true;
}

bool KittiPreprocessNode::has_gnss_data()
{
  if (gnss_data_buffer_.size() == 0) {
    return false;
  }
  if (twist_data_buffer_.size() == 0) {
    return false;
  }
  if (imu_data_buffer_.size() == 0) {
    return false;
  }
  return true;
}

bool KittiPreprocessNode::valid_gnss_data()
{
  current_gnss_data_ = gnss_data_buffer_.front();
  current_twist_data_ = twist_data_buffer_.front();
  current_imu_data_ = imu_data_buffer_.front();
  double diff_twist_time = current_gnss_data_.time - current_twist_data_.time;
  double diff_imu_time = current_gnss_data_.time - current_imu_data_.time;
  if (diff_twist_time < -0.05 || diff_imu_time < -0.05) {
    std::cout << "drop gnss data" << std::endl;
    gnss_data_buffer_.pop_front();
    return false;
  }
  if (diff_twist_time > 0.05) {
    std::cout << "drop twist data" << std::endl;
    twist_data_buffer_.pop_front();
    return false;
  }
  if (diff_imu_time > 0.05) {
    std::cout << "drop imu data" << std::endl;
    imu_data_buffer_.pop_front();
    return false;
  }
  imu_data_buffer_.pop_front();
  twist_data_buffer_.pop_front();
  gnss_data_buffer_.pop_front();
  return true;
}

bool KittiPreprocessNode::get_synced_gnss(double time, localization_common::OdomData & odom)
{
  if (gnss_odom_buffer_.empty() || gnss_odom_buffer_.back().time < time) {
    return false;
  }
  if (gnss_odom_buffer_.front().time > time) {
    return false;
  }
  if (gnss_odom_buffer_.size() == 1) {
    odom = gnss_odom_buffer_.at(1);
    return true;
  }
  while (gnss_odom_buffer_.at(1).time < time) {
    gnss_odom_buffer_.pop_front();
  }
  if (gnss_odom_buffer_.at(1).time == time) {
    odom = gnss_odom_buffer_.at(1);
    gnss_odom_buffer_.pop_front();
  } else {
    odom = interpolate_odom(gnss_odom_buffer_.at(0), gnss_odom_buffer_.at(1), time);
  }
  return true;
}

}  // namespace localization_common
