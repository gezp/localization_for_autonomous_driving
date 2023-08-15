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
#include <string>
//
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
//
#include "localization_interfaces/srv/save_odometry.hpp"
#include "localization_common/publisher/cloud_publisher.hpp"
#include "localization_common/publisher/odometry_publisher.hpp"
#include "localization_common/subscriber/cloud_subscriber.hpp"
#include "localization_common/subscriber/odometry_subscriber.hpp"
#include "localization_common/tf_utils.hpp"
#include "lidar_odometry/lidar_odometry.hpp"

namespace lidar_odometry
{

class LidarOdometryNode
{
public:
  explicit LidarOdometryNode(rclcpp::Node::SharedPtr node);
  ~LidarOdometryNode();

private:
  bool run();
  bool update();
  bool publish_data();

private:
  rclcpp::Node::SharedPtr node_;
  // tf
  std::string base_link_frame_id_{"base_link"};
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  bool publish_tf_{false};
  // pub & sub
  std::shared_ptr<localization_common::CloudSubscriber<pcl::PointXYZ>> cloud_sub_;
  std::shared_ptr<localization_common::OdometrySubscriber> reference_odom_sub_;
  std::shared_ptr<localization_common::CloudPublisher<pcl::PointXYZ>> cloud_pub_;
  std::shared_ptr<localization_common::CloudPublisher<pcl::PointXYZ>> local_map_pub_;
  std::shared_ptr<localization_common::OdometryPublisher> lidar_odom_pub_;
  // front end tool and thread
  std::shared_ptr<LidarOdometry> lidar_odometry_;
  std::unique_ptr<std::thread> run_thread_;
  bool exit_{false};
  // data
  std::deque<localization_common::LidarData<pcl::PointXYZ>> lidar_data_buff_;
  std::deque<localization_common::OdomData> ref_pose_data_buff_;
  localization_common::LidarData<pcl::PointXYZ> current_lidar_data_;
  // init pose
  Eigen::Matrix4d initial_pose_ = Eigen::Matrix4d::Identity();
  bool use_initial_pose_from_topic_{false};
  bool inited_{false};
};

}  // namespace lidar_odometry
