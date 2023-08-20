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

bool LidarImuFusionNode::has_imu_data() {return !raw_imu_buffer_.empty();}

bool LidarImuFusionNode::has_lidar_data() {return !lidar_pose_buffer_.empty();}

bool LidarImuFusionNode::read_data()
{
  imu_raw_sub_->parse_data(raw_imu_buffer_);
  gnss_sub_->parse_data(gnss_pose_buffer_);
  lidar_pose_sub_->parse_data(lidar_pose_buffer_);
  return true;
}

bool LidarImuFusionNode::valid_lidar_data()
{
  current_lidar_pose_ = lidar_pose_buffer_.front();
  current_gnss_pose_ = gnss_pose_buffer_.front();
  double diff_gnss_time = current_lidar_pose_.time - current_gnss_pose_.time;
  if (diff_gnss_time < -0.05) {
    lidar_pose_buffer_.pop_front();
    return false;
  }
  if (diff_gnss_time > 0.05) {
    gnss_pose_buffer_.pop_front();
    return false;
  }
  lidar_pose_buffer_.pop_front();
  gnss_pose_buffer_.pop_front();
  return true;
}

bool LidarImuFusionNode::run()
{
  // get extrinsics
  if (!is_valid_extrinsics_) {
    if (!extrinsics_manager_->lookup(base_frame_id_, imu_frame_id_, T_base_imu_)) {
      return false;
    }
    fusion_->set_extrinsic(T_base_imu_);
    is_valid_extrinsics_ = true;
  }
  read_data();
  // check inited
  if (!fusion_->has_inited()) {
    if (raw_imu_buffer_.empty() || lidar_pose_buffer_.size() < 2) {
      return false;
    }
    if (raw_imu_buffer_.back().time < lidar_pose_buffer_.front().time) {
      // wait imu data
      return false;
    }
    // get lidar data
    if (!valid_lidar_data()) {
      std::cout << "sync data failed" << std::endl;
      return false;
    }
    if (current_lidar_pose_.time < raw_imu_buffer_.front().time) {
      std::cout << "drop older lidar data: " << current_lidar_pose_.time << std::endl;
      // old lidar data, drop
      return false;
    }
    // get sync imu
    localization_common::ImuData imu;
    while (raw_imu_buffer_.front().time < current_gnss_pose_.time) {
      imu = raw_imu_buffer_.front();
      raw_imu_buffer_.pop_front();
    }
    auto & next_imu = raw_imu_buffer_.front();
    auto sync_imu = localization_common::interpolate_imu(imu, next_imu, current_gnss_pose_.time);
    // init_pose: pose of imu body in map frame
    // init_vel: linear velocity of imu body in map frame
    auto odom = localization_common::transform_odom(current_gnss_pose_, T_base_imu_);
    Eigen::Matrix4d init_pose = odom.pose;
    Eigen::Vector3d init_vel = init_pose.block<3, 3>(0, 0) * odom.linear_velocity;
    fusion_->init(init_pose, init_vel, sync_imu);
    publish_fusion_odom();
    std::cout << "Localization Init Succeeded at " << current_gnss_pose_.time << std::endl
              << "Init Position: " << init_pose.block<3, 1>(0, 3).transpose() << std::endl
              << "Init Velocity: " << init_vel.transpose() << std::endl;
    return true;
  }
  // process data
  if (has_imu_data()) {
    localization_common::ImuData imu = raw_imu_buffer_.front();
    raw_imu_buffer_.pop_front();
    if (fusion_->add_imu_data(imu)) {
      publish_fusion_odom();
    }
  }
  if (has_lidar_data() && valid_lidar_data()) {
    if (fusion_->add_observation_data(current_lidar_pose_)) {
      publish_fusion_odom();
    }
  }
  return true;
}

bool LidarImuFusionNode::publish_fusion_odom()
{
  auto odom = fusion_->get_current_odom();
  // publish tf
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = localization_common::to_ros_time(odom.time);
  msg.header.frame_id = "map";
  msg.child_frame_id = base_frame_id_;
  msg.transform = localization_common::to_transform_msg(odom.pose);
  tf_pub_->sendTransform(msg);
  // publish fusion odometry
  fused_odom_pub_->publish(odom);
  return true;
}

}  // namespace kf_based_localization
