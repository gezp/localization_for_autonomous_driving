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

#include "localization_common/data_synchronization.hpp"
#include "localization_common/sensor_data/lidar_data.hpp"
#include "localization_common/sensor_data_utils.hpp"

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
  imu_sub_ = std::make_shared<ImuSubscriber>(node, "/kitti/oxts/imu", 10000);
  twist_sub_ = std::make_shared<TwistSubscriber>(node, "/kitti/oxts/gps/vel", 10000);
  nav_sat_fix_sub_ = std::make_shared<NavSatFixSubscriber>(node, "/kitti/oxts/gps/fix", 10000);
  if (use_manual_map_origin_) {
    nav_sat_fix_sub_->set_map_origin(map_origin_[0], map_origin_[1], map_origin_[2]);
  }
  // publisher
  cloud_pub_ =
    std::make_shared<CloudPublisher<pcl::PointXYZ>>(node, "synced_cloud", base_frame_id_, 100);
  gnss_pose_pub_ =
    std::make_shared<OdometryPublisher>(node, "synced_gnss/pose", "map", base_frame_id_, 100);
  imu_pub_ = std::make_shared<ImuPublisher>(node, "synced_imu", imu_frame_id_, 100);
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
  if (!read_data()) {
    return false;
  }
  while (has_data()) {
    if (!valid_data()) {
      continue;
    }
    transform_data();
    publish_data();
  }
  return true;
}

bool KittiPreprocessNode::read_data()
{
  static std::deque<ImuData2> unsynced_imu_;
  static std::deque<TwistData> unsynced_twist_;
  static std::deque<GnssData> unsynced_gnss_;

  // fetch lidar measurements from buffer:
  cloud_sub_->parse_data(lidar_data_buff_);
  imu_sub_->parse_data(unsynced_imu_);
  twist_sub_->parse_data(unsynced_twist_);
  nav_sat_fix_sub_->parse_data(unsynced_gnss_);

  static bool sensor_inited = false;
  if (!sensor_inited && lidar_data_buff_.size() < 5) {
    return false;
  }

  if (lidar_data_buff_.size() == 0) {
    return false;
  }

  // use timestamp of lidar measurement as reference:
  double cloud_time = lidar_data_buff_.front().time;
  // sync IMU, velocity and GNSS with lidar measurement:
  // find the two closest measurement around lidar measurement time
  // then use linear interpolation to generate synced measurement:
  bool valid_imu = sync_imu_data2(unsynced_imu_, imu_data_buff_, cloud_time);
  bool valid_velocity = sync_twist_data(unsynced_twist_, twist_data_buff_, cloud_time);
  bool valid_gnss = sync_gnss_data(unsynced_gnss_, gnss_data_buff_, cloud_time);

  // only mark lidar as 'inited' when all the three sensors are synced:
  if (!sensor_inited) {
    if (!valid_imu || !valid_velocity || !valid_gnss) {
      lidar_data_buff_.pop_front();
      return false;
    }
    sensor_inited = true;
  }

  return true;
}

bool KittiPreprocessNode::has_data()
{
  if (lidar_data_buff_.size() == 0) {
    return false;
  }
  if (imu_data_buff_.size() == 0) {
    return false;
  }
  if (twist_data_buff_.size() == 0) {
    return false;
  }
  if (gnss_data_buff_.size() == 0) {
    return false;
  }
  return true;
}

bool KittiPreprocessNode::valid_data()
{
  current_lidar_data_ = lidar_data_buff_.front();
  current_imu_data_ = imu_data_buff_.front();
  current_twist_data_ = twist_data_buff_.front();
  current_gnss_data_ = gnss_data_buff_.front();

  double diff_imu_time = current_lidar_data_.time - current_imu_data_.time;
  double diff_twist_time = current_lidar_data_.time - current_twist_data_.time;
  double diff_gnss_time = current_lidar_data_.time - current_gnss_data_.time;
  if (diff_imu_time < -0.05 || diff_twist_time < -0.05 || diff_gnss_time < -0.05) {
    lidar_data_buff_.pop_front();
    return false;
  }
  if (diff_imu_time > 0.05) {
    imu_data_buff_.pop_front();
    return false;
  }
  if (diff_twist_time > 0.05) {
    twist_data_buff_.pop_front();
    return false;
  }
  if (diff_gnss_time > 0.05) {
    gnss_data_buff_.pop_front();
    return false;
  }
  lidar_data_buff_.pop_front();
  imu_data_buff_.pop_front();
  twist_data_buff_.pop_front();
  gnss_data_buff_.pop_front();
  return true;
}

bool KittiPreprocessNode::transform_data()
{
  // motion compensation for lidar measurements:
  auto lidar_twist = transform_twist(current_twist_data_, T_imu_lidar_);
  distortion_adjust_->set_motion_info(0.1, lidar_twist);
  distortion_adjust_->adjust_cloud(
    current_lidar_data_.point_cloud, current_lidar_data_.point_cloud);
  Eigen::Matrix4d pose = T_base_lidar_.inverse();
  pcl::transformPointCloud(
    *current_lidar_data_.point_cloud, *current_lidar_data_.point_cloud, pose);
  // get reference pose (position from GNSS, orientation from IMU)
  gnss_pose_ = Eigen::Matrix4d::Identity();
  gnss_pose_.block<3, 1>(0, 3) = current_gnss_data_.antenna_position;
  gnss_pose_.block<3, 3>(0, 0) = current_imu_data_.orientation.matrix();
  gnss_pose_ = gnss_pose_ * T_imu_base_;
  return true;
}

bool KittiPreprocessNode::publish_data()
{
  auto twist = transform_twist(current_twist_data_, T_imu_base_);
  cloud_pub_->publish(current_lidar_data_.point_cloud, current_lidar_data_.time);
  OdomData odom;
  odom.time = current_lidar_data_.time;
  odom.pose = gnss_pose_;
  odom.linear_velocity = twist.linear_velocity;
  odom.angular_velocity = twist.angular_velocity;
  gnss_pose_pub_->publish(odom);
  ImuData imu_data;
  imu_data.time = current_lidar_data_.time;
  imu_data.linear_acceleration = current_imu_data_.linear_acceleration;
  imu_data.angular_velocity = current_imu_data_.angular_velocity;
  imu_pub_->publish(imu_data);
  return true;
}
}  // namespace localization_common
