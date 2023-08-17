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

#include "kf_based_localization/lidar_imu_fusion_node.hpp"

#include <filesystem>
#include <fstream>
#include <ostream>

#include "localization_common/sensor_data_utils.hpp"

namespace kf_based_localization
{

LidarImuFusionNode::LidarImuFusionNode(rclcpp::Node::SharedPtr node)
{
  std::string config_file;
  node->declare_parameter("config_file", config_file);
  node->declare_parameter("base_frame_id", base_frame_id_);
  node->declare_parameter("imu_frame_id", imu_frame_id_);
  node->get_parameter("config_file", config_file);
  node->get_parameter("base_frame_id", base_frame_id_);
  node->get_parameter("imu_frame_id", imu_frame_id_);
  std::cout << "config file path:" << config_file << std::endl;
  // subscriber:
  imu_raw_sub_ =
    std::make_shared<localization_common::ImuSubscriber>(node, "/kitti/oxts/imu/extract", 100000);
  // gnss/pose in map frame:
  gnss_sub_ =
    std::make_shared<localization_common::OdometrySubscriber>(node, "synced_gnss/pose", 10000);
  // lidar/pose in map frame:
  lidar_pose_sub_ = std::make_shared<localization_common::OdometrySubscriber>(
    node, "localization/lidar/pose", 10000);
  // fused pose in map frame:
  fused_odom_pub_ = std::make_shared<localization_common::OdometryPublisher>(
    node, "localization/fused/pose", "map", base_frame_id_, 100);
  // tf
  tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  // extrinsics
  extrinsics_manager_ = std::make_shared<localization_common::ExtrinsicsManager>(node);
  extrinsics_manager_->enable_tf_listener();
  // fusion module
  YAML::Node config_node = YAML::LoadFile(config_file);
  fusion_ = std::make_shared<LidarImuFusion>();
  std::cout << "---------Init IMU-Lidar Fusion for Localization------------" << std::endl;
  fusion_->init_config(config_node);
  // thread
  run_thread_ = std::make_unique<std::thread>(
    [this]() {
      while (!exit_) {
        if (!run()) {
          using namespace std::chrono_literals;
          std::this_thread::sleep_for(5ms);
        }
      }
    });
}

LidarImuFusionNode::~LidarImuFusionNode()
{
  exit_ = true;
  if (run_thread_) {
    run_thread_->join();
  }
}

bool LidarImuFusionNode::has_imu_data() {return !imu_raw_data_buff_.empty();}

bool LidarImuFusionNode::has_lidar_data() {return !lidar_pose_data_buff_.empty();}

bool LidarImuFusionNode::read_data()
{
  imu_raw_sub_->parse_data(imu_raw_data_buff_);
  gnss_sub_->parse_data(gnss_data_buff_);
  lidar_pose_sub_->parse_data(lidar_pose_data_buff_);
  return true;
}

bool LidarImuFusionNode::valid_lidar_data()
{
  current_lidar_pose_data_ = lidar_pose_data_buff_.front();
  current_gnss_data_ = gnss_data_buff_.front();
  double diff_gnss_time = current_lidar_pose_data_.time - current_gnss_data_.time;
  if (diff_gnss_time < -0.05) {
    lidar_pose_data_buff_.pop_front();
    return false;
  }
  if (diff_gnss_time > 0.05) {
    gnss_data_buff_.pop_front();
    return false;
  }
  lidar_pose_data_buff_.pop_front();
  gnss_data_buff_.pop_front();
  return true;
}

bool LidarImuFusionNode::has_valid_lidar_data()
{
  if (imu_raw_data_buff_.back().time < lidar_pose_data_buff_.front().time) {
    // wait imu data
    return false;
  }
  // get lidar data
  if (!valid_lidar_data()) {
    std::cout << "sync data failed" << std::endl;
    return false;
  }
  if (current_lidar_pose_data_.time < imu_raw_data_buff_.front().time) {
    std::cout << "drop older lidar data: " << current_lidar_pose_data_.time << std::endl;
    // old lidar data, drop
    return false;
  }
  return true;
}

bool LidarImuFusionNode::run()
{
  // get extrinsics
  if (!is_valid_extrinsics_) {
    if (!extrinsics_manager_->lookup(base_frame_id_, imu_frame_id_, T_base_imu_)) {
      return false;
    }
    is_valid_extrinsics_ = true;
  }
  read_data();
  // check inited
  if (!fusion_->has_inited()) {
    if (imu_raw_data_buff_.empty() || lidar_pose_data_buff_.size() < 2) {
      return false;
    }
    if (!has_valid_lidar_data()) {
      return false;
    }
    while (imu_raw_data_buff_.front().time < current_gnss_data_.time) {
      current_imu_raw_data_ = imu_raw_data_buff_.front();
      imu_raw_data_buff_.pop_front();
    }
    current_imu_synced_data_ = localization_common::interpolate_imu(
      current_imu_raw_data_, imu_raw_data_buff_.front(), current_gnss_data_.time);
    // init_pose: pose of imu body in map frame
    // init_vel: linear velocity of imu body in map frame
    auto odom = localization_common::transform_odom(current_gnss_data_, T_base_imu_);
    Eigen::Matrix4d init_pose = odom.pose;
    Eigen::Vector3d init_vel = init_pose.block<3, 3>(0, 0) * odom.linear_velocity;
    fusion_->init(init_pose, init_vel, current_imu_synced_data_);
    publish_fusion_odom();
    std::cout << "Localization Init Succeeded at " << current_gnss_data_.time << std::endl
              << "Init Position: " << init_pose.block<3, 1>(0, 3).transpose() << std::endl
              << "Init Velocity: " << init_vel.transpose() << std::endl;
    return true;
  }
  // check
  if (!has_imu_data()) {
    return false;
  }
  if (has_lidar_data()) {
    // correct
    if (!has_valid_lidar_data()) {
      return false;
    }
    while (imu_raw_data_buff_.front().time < current_lidar_pose_data_.time) {
      current_imu_raw_data_ = imu_raw_data_buff_.front();
      imu_raw_data_buff_.pop_front();
      update_localization();
    }
    correct_localization();
  } else {
    // predict
    double predict_dt = correct_time_ + 0.09;
    if (imu_raw_data_buff_.front().time > predict_dt) {
      return false;
    }
    current_imu_raw_data_ = imu_raw_data_buff_.front();
    imu_raw_data_buff_.pop_front();
    update_localization();
  }
  return true;
}

bool LidarImuFusionNode::update_localization()
{
  // drop older imu data
  if (current_imu_raw_data_.time < fusion_->get_time()) {
    std::cout << "drop older imu data: " << current_imu_raw_data_.time << std::endl;
    return false;
  }
  if (!fusion_->process_imu_data(current_imu_raw_data_)) {
    std::cout << "update_localization failed." << std::endl;
    return false;
  }
  current_imu_data_ = current_imu_raw_data_;
  publish_fusion_odom();
  return true;
}

bool LidarImuFusionNode::correct_localization()
{
  // check time
  if (current_lidar_pose_data_.time < fusion_->get_time()) {
    std::cout << "Observation has a older timestamp. Skip." << std::endl;
  }
  // update imu data
  current_imu_synced_data_ = localization_common::interpolate_imu(
    current_imu_raw_data_, imu_raw_data_buff_.front(), current_lidar_pose_data_.time);
  if (!fusion_->process_imu_data(current_imu_synced_data_)) {
    std::cout << "correct_localization failed [process_imu_data]." << std::endl;
    return false;
  }
  // imu body in map frame
  localization_common::OdomData lidar_pose = current_lidar_pose_data_;
  lidar_pose.pose = lidar_pose.pose * T_base_imu_;
  if (!fusion_->process_lidar_data(lidar_pose)) {
    std::cout << "correct_localization failed [process_lidar_data]." << std::endl;
    return false;
  }
  current_imu_data_ = current_imu_synced_data_;
  publish_fusion_odom();
  correct_time_ = current_lidar_pose_data_.time;
  return true;
}

bool LidarImuFusionNode::publish_fusion_odom()
{
  auto nav_state = fusion_->get_imu_nav_state();
  // odometry for imu frame
  localization_common::OdomData odom_imu;
  odom_imu.time = nav_state.time;
  odom_imu.pose.block<3, 1>(0, 3) = nav_state.position;
  odom_imu.pose.block<3, 3>(0, 0) = nav_state.orientation;
  odom_imu.linear_velocity = nav_state.linear_velocity;
  odom_imu.angular_velocity = current_imu_data_.angular_velocity;
  // odometry for base frame
  auto odom = localization_common::transform_odom(odom_imu, T_base_imu_.inverse());
  // publish tf
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = localization_common::to_ros_time(nav_state.time);
  msg.header.frame_id = "map";
  msg.child_frame_id = base_frame_id_;
  msg.transform = localization_common::to_transform_msg(odom.pose);
  tf_pub_->sendTransform(msg);
  // publish fusion odometry
  fused_odom_pub_->publish(odom);
  return true;
}

}  // namespace kf_based_localization
