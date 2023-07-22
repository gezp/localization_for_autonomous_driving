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

#pragma once

#include <memory>
#include <deque>
#include <fstream>
#include <string>
//
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
//
#include "localization_interfaces/srv/optimize_map.hpp"
#include "localization_interfaces/srv/save_map.hpp"
#include "localization_common/subscriber/cloud_subscriber.hpp"
#include "localization_common/subscriber/loop_pose_subscriber.hpp"
#include "localization_common/subscriber/odometry_subscriber.hpp"
#include "localization_common/subscriber/imu_subscriber.hpp"
#include "localization_common/publisher/cloud_publisher.hpp"
#include "localization_common/publisher/key_frame_publisher.hpp"
#include "localization_common/publisher/path_publisher.hpp"
#include "localization_common/publisher/odometry_publisher.hpp"
#include "localization_common/tf_utils.hpp"
#include "loosely_lio_mapping/lio_back_end.hpp"
#include "lidar_mapping/map_generator.hpp"

namespace loosely_lio_mapping
{
class LioBackEndNode
{
public:
  explicit LioBackEndNode(rclcpp::Node::SharedPtr node);
  ~LioBackEndNode();

private:
  bool run();
  bool force_optimize();
  bool read_data();
  bool maybe_insert_loop_pose();
  bool has_data();
  bool valid_data();
  bool init_calibration();
  bool update_imu_pre_integration();
  bool update_back_end();
  bool publish_data();
  void save_pose(std::ofstream & ofs, const Eigen::Matrix4f & pose);
  bool save_trajectory(const std::deque<localization_common::KeyFrame> & optimized_key_frames);

private:
  // sub
  std::shared_ptr<localization_common::CloudSubscriber<pcl::PointXYZ>> cloud_sub_;
  std::shared_ptr<localization_common::OdometrySubscriber> gnss_pose_sub_;
  std::shared_ptr<localization_common::OdometrySubscriber> lidar_odom_sub_;
  std::shared_ptr<localization_common::LoopPoseSubscriber> loop_pose_sub_;
  std::shared_ptr<localization_common::ImuSubscriber> imu_raw_sub_;
  std::shared_ptr<localization_common::ImuSubscriber> imu_synced_sub_;
  // pub
  std::shared_ptr<localization_common::CloudPublisher<pcl::PointXYZ>> key_scan_pub_;
  std::shared_ptr<localization_common::KeyFramePublisher> key_frame_pub_;
  std::shared_ptr<localization_common::KeyFramePublisher> key_gnss_pub_;
  std::shared_ptr<localization_common::PathPublisher> optimized_path_pub_;
  std::shared_ptr<localization_common::OdometryPublisher> optimized_odom_pub_;
  std::shared_ptr<localization_common::CloudPublisher<pcl::PointXYZ>> global_map_pub_;
  // tf
  std::string imu_frame_id_{"imu"};
  std::string base_link_frame_id_{"base_link"};
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  Eigen::Matrix4f base_link_to_imu_ = Eigen::Matrix4f::Identity();
  bool publish_tf_{false};
  // srv
  rclcpp::Service<localization_interfaces::srv::OptimizeMap>::SharedPtr optimize_map_srv_;
  bool need_optimize_map_{false};
  rclcpp::Service<localization_interfaces::srv::SaveMap>::SharedPtr save_map_srv_;
  bool save_map_flag_{false};
  // back end and flow thread
  std::shared_ptr<LioBackEnd> back_end_;
  // map generator
  std::shared_ptr<lidar_mapping::MapGenerator> map_generator_;
  std::unique_ptr<std::thread> run_thread_;
  bool exit_{false};
  // data
  std::deque<localization_common::LidarData<pcl::PointXYZ>> lidar_data_buff_;
  std::deque<localization_common::PoseData> gnss_pose_data_buff_;
  std::deque<localization_common::PoseData> lidar_odom_data_buff_;
  std::deque<localization_common::LoopPose> loop_pose_data_buff_;
  std::deque<localization_common::ImuData> imu_raw_data_buff_;
  std::deque<localization_common::ImuData> imu_synced_data_buff_;

  localization_common::LidarData<pcl::PointXYZ> current_lidar_data_;
  localization_common::PoseData current_gnss_pose_data_;
  localization_common::PoseData current_lidar_odom_data_;
  localization_common::ImuData current_imu_data_;
  // trajectory for evo evaluation:
  std::string trajectory_path_ = "";
  struct
  {
    size_t length = 0;
    std::deque<double> time;
    std::deque<Eigen::Matrix4f> lidar;
    std::deque<Eigen::Matrix4f> ref;
  } trajectory_;
};
}  // namespace loosely_lio_mapping
