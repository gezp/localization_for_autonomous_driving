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

namespace kf_based_localization
{

LidarImuFusionNode::LidarImuFusionNode(rclcpp::Node::SharedPtr node)
{
  std::string config_file;
  node->declare_parameter("config_file", config_file);
  node->get_parameter("config_file", config_file);
  std::cout << "config file path:" << config_file << std::endl;
  // subscriber:
  imu_raw_sub_ =
    std::make_shared<localization_common::ImuSubscriber>(node, "/kitti/oxts/imu/extract", 100000);
  imu_synced_sub_ = std::make_shared<localization_common::ImuSubscriber>(node, "synced_imu", 10000);
  // synced GNSS-odo measurement:
  pos_vel_sub_ =
    std::make_shared<localization_common::PosVelSubscriber>(node, "synced_pos_vel", 10000);
  // gnss/pose in map frame:
  gnss_sub_ =
    std::make_shared<localization_common::OdometrySubscriber>(node, "synced_gnss/pose", 10000);
  // lidar/pose in map frame:
  lidar_pose_sub_ = std::make_shared<localization_common::OdometrySubscriber>(
    node, "localization/lidar/pose", 10000);
  // fused pose in map frame:
  fused_odom_pub_ = std::make_shared<localization_common::OdometryPublisher>(
    node, "localization/fused/pose", "map", "base_link", 100);
  // tf:
  imu_frame_id_ = "imu_link";
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
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
          std::this_thread::sleep_for(10ms);
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

bool LidarImuFusionNode::has_lidar_data()
{
  return !imu_synced_data_buff_.empty() && !pos_vel_data_buff_.empty() &&
         !lidar_pose_data_buff_.empty();
}

bool LidarImuFusionNode::read_data()
{
  imu_raw_sub_->parse_data(imu_raw_data_buff_);
  imu_synced_sub_->parse_data(imu_synced_data_buff_);
  pos_vel_sub_->parse_data(pos_vel_data_buff_);
  gnss_sub_->parse_data(gnss_data_buff_);
  lidar_pose_sub_->parse_data(lidar_pose_data_buff_);
  return true;
}

bool LidarImuFusionNode::has_data()
{
  // check raw imu
  if (fusion_->has_inited() && has_imu_data()) {
    return true;
  }
  return has_lidar_data();
}

bool LidarImuFusionNode::valid_lidar_data()
{
  current_lidar_pose_data_ = lidar_pose_data_buff_.front();
  current_imu_synced_data_ = imu_synced_data_buff_.front();
  current_pos_vel_data_ = pos_vel_data_buff_.front();
  double diff_imu_time = current_lidar_pose_data_.time - current_imu_synced_data_.time;
  double diff_pos_vel_time = current_lidar_pose_data_.time - current_pos_vel_data_.time;
  if (diff_imu_time < -0.05 || diff_pos_vel_time < -0.05) {
    lidar_pose_data_buff_.pop_front();
    return false;
  }
  if (diff_imu_time > 0.05) {
    imu_synced_data_buff_.pop_front();
    return false;
  }
  if (diff_pos_vel_time > 0.05) {
    pos_vel_data_buff_.pop_front();
    return false;
  }
  imu_synced_data_buff_.pop_front();
  pos_vel_data_buff_.pop_front();
  lidar_pose_data_buff_.pop_front();
  return true;
}

bool LidarImuFusionNode::init_calibration()
{
  // lookup imu pose in lidar frame:
  static bool calibration_received = false;
  if (!calibration_received) {
    if (localization_common::lookup_in_tf_buffer(
        tf_buffer_, imu_frame_id_, base_link_frame_id_, base_link_to_imu_))
    {
      calibration_received = true;
    }
  }
  return calibration_received;
}

bool LidarImuFusionNode::run()
{
  if (!init_calibration()) {
    return false;
  }
  read_data();
  if (!has_data()) {
    return false;
  }
  // check inited
  if (!fusion_->has_inited()) {
    if (valid_lidar_data()) {
      // init_pose: pose of imu body in map frame
      // init_vel: linear velocity of imu body in map frame
      Eigen::Matrix4f init_pose = current_lidar_pose_data_.pose.cast<float>() *
        base_link_to_imu_.inverse();
      Eigen::Vector3f init_vel = init_pose.block<3, 3>(0, 0) * current_pos_vel_data_.vel;
      if (fusion_->init(init_pose, init_vel, current_imu_synced_data_)) {
        publish_fusion_odom();
        std::cout << "Localization Init Succeeded at " << current_imu_synced_data_.time << std::endl
                  << "Init Position: " << init_pose(0, 3) << ", " << init_pose(1, 3) << ", "
                  << init_pose(2, 3) << std::endl
                  << "Init Velocity: " << init_vel.x() << ", " << init_vel.y() << ", "
                  << init_vel.z() << std::endl;
        return true;
      }
    }
    return false;
  }
  // remove older imu data
  while (has_imu_data() && imu_raw_data_buff_.front().time < fusion_->get_time()) {
    imu_raw_data_buff_.pop_front();
  }
  // correct
  if (has_lidar_data() && valid_lidar_data()) {
    while (has_imu_data() && imu_raw_data_buff_.front().time < current_lidar_pose_data_.time) {
      current_imu_raw_data_ = imu_raw_data_buff_.front();
      imu_raw_data_buff_.pop_front();
      update_localization();
    }
    correct_localization();
  }
  // predict
  double predict_dt = current_lidar_pose_data_.time + 0.09;
  if (!has_lidar_data() && has_imu_data()) {
    if (imu_raw_data_buff_.front().time < predict_dt) {
      current_imu_raw_data_ = imu_raw_data_buff_.front();
      imu_raw_data_buff_.pop_front();
      update_localization();
    } else {
      return false;
    }
  }
  return true;
}

bool LidarImuFusionNode::update_localization()
{
  if (!fusion_->process_imu_data(current_imu_raw_data_)) {
    std::cout << "update_localization failed." << std::endl;
    return false;
  }
  publish_fusion_odom();
  return true;
}

bool LidarImuFusionNode::correct_localization()
{
  // check time:
  if (fusion_->get_time() > current_imu_synced_data_.time) {
    std::cout << "Observation has a older timestamp. Skip." << std::endl;
  }
  // update imu data
  if (!fusion_->process_imu_data(current_imu_synced_data_)) {
    std::cout << "correct_localization failed [process_imu_data]." << std::endl;
    return false;
  }
  // imu body in map frame
  localization_common::OdomData lidar_pose = current_lidar_pose_data_;
  lidar_pose.pose = lidar_pose.pose * base_link_to_imu_.inverse().cast<double>();
  if (!fusion_->process_lidar_data(lidar_pose)) {
    std::cout << "correct_localization failed [process_lidar_data]." << std::endl;
    return false;
  }
  publish_fusion_odom();
  return true;
}

bool LidarImuFusionNode::publish_fusion_odom()
{
  // fused_pose in map frame, fused_vel in imu frame
  // TODO(gezp) : move fused_vel to base_link frame.
  auto nav_state = fusion_->get_imu_nav_state();
  Eigen::Matrix4f fused_pose = Eigen::Matrix4f::Identity();
  fused_pose.block<3, 1>(0, 3) = nav_state.position.cast<float>();
  fused_pose.block<3, 3>(0, 0) = nav_state.orientation.cast<float>();
  Eigen::Vector3f fused_vel = nav_state.linear_velocity.cast<float>();
  fused_pose = fused_pose * base_link_to_imu_;
  fused_vel = fused_pose.block<3, 3>(0, 0).transpose() * fused_vel;
  // publish tf:
  auto msg = localization_common::to_transform_stamped_msg(fused_pose, nav_state.time);
  msg.header.frame_id = "map";
  msg.child_frame_id = "base_link";
  tf_pub_->sendTransform(msg);
  // publish fusion odometry:
  localization_common::OdomData odom;
  odom.time = nav_state.time;
  odom.pose = fused_pose.cast<double>();
  odom.linear_velocity = fused_vel.cast<double>();
  fused_odom_pub_->publish(odom);
  return true;
}

}  // namespace kf_based_localization
