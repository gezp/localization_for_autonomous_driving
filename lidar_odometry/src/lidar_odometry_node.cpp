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

#include "lidar_odometry/lidar_odometry_node.hpp"

#include <filesystem>

namespace lidar_odometry
{
LidarOdometryNode::LidarOdometryNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  // lidar_odometry_config
  std::string lidar_odometry_config;
  std::vector<double> pose{0, 0, 0, 0, 0, 0, 1};
  node->declare_parameter("lidar_odometry_config", lidar_odometry_config);
  node->declare_parameter("publish_tf", publish_tf_);
  node->declare_parameter("initial_pose", pose);
  node->declare_parameter("use_initial_pose_from_topic", use_initial_pose_from_topic_);
  node->declare_parameter("base_frame_id", base_frame_id_);
  node->declare_parameter("lidar_frame_id", lidar_frame_id_);
  node->get_parameter("lidar_odometry_config", lidar_odometry_config);
  node->get_parameter("publish_tf", publish_tf_);
  node->get_parameter("initial_pose", pose);
  node->get_parameter("use_initial_pose_from_topic", use_initial_pose_from_topic_);
  node->get_parameter("base_frame_id", base_frame_id_);
  node->get_parameter("lidar_frame_id", lidar_frame_id_);
  if (pose.size() == 7) {
    initial_pose_.block<3, 1>(0, 3) = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    initial_pose_.block<3, 3>(0, 0) =
      Eigen::Quaterniond(pose[6], pose[3], pose[4], pose[5]).matrix();
  }
  RCLCPP_INFO(node->get_logger(), "lidar_odometry_config: [%s]", lidar_odometry_config.c_str());
  if (lidar_odometry_config == "" || (!std::filesystem::exists(lidar_odometry_config))) {
    RCLCPP_FATAL(node->get_logger(), "lidar_odometry_config is invalid");
    return;
  }
  // init front end
  lidar_odometry_ = std::make_shared<LidarOdometry>();
  lidar_odometry_->init_config(lidar_odometry_config);
  // init sub & pub:
  cloud_sub_ = std::make_shared<localization_common::CloudSubscriber<pcl::PointXYZ>>(
    node, "synced_cloud", 10000);
  if (use_initial_pose_from_topic_) {
    reference_odom_sub_ =
      std::make_shared<localization_common::OdometrySubscriber>(node, "reference_odom", 10000);
  }
  cloud_pub_ = std::make_shared<localization_common::CloudPublisher<pcl::PointXYZ>>(
    node, "lidar_odometry/current_scan", "map", 100);
  local_map_pub_ = std::make_shared<localization_common::CloudPublisher<pcl::PointXYZ>>(
    node, "lidar_odometry/local_map", "map", 100);
  lidar_odom_pub_ = std::make_shared<localization_common::OdometryPublisher>(
    node, "lidar_odom", "map", base_frame_id_, 100);
  tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  // extrinsics
  extrinsics_manager_ = std::make_shared<localization_common::ExtrinsicsManager>(node);
  extrinsics_manager_->enable_tf_listener();
  // process loop callback
  run_thread_ = std::make_unique<std::thread>(
    [this]() {
      while (!exit_) {
        run();
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(10ms);
      }
    });
}

LidarOdometryNode::~LidarOdometryNode()
{
  exit_ = true;
  if (run_thread_) {
    run_thread_->join();
  }
}

bool LidarOdometryNode::run()
{
  // get extrinsics
  if (!is_valid_extrinsics_) {
    if (!extrinsics_manager_->lookup(base_frame_id_, lidar_frame_id_, T_base_lidar_)) {
      return false;
    }
    lidar_odometry_->set_extrinsic(T_base_lidar_);
    is_valid_extrinsics_ = true;
  }
  cloud_sub_->parse_data(lidar_data_buff_);
  while (lidar_data_buff_.size() != 0) {
    current_lidar_data_ = lidar_data_buff_.front();
    lidar_data_buff_.pop_front();
    if (!update()) {
      RCLCPP_WARN(node_->get_logger(), "Update Odometry failed!");
      continue;
    }
    publish_data();
  }
  return true;
}

bool LidarOdometryNode::get_sync_reference_odom(localization_common::OdomData & odom)
{
  localization_common::OdomData current_ref;
  while (true) {
    // get reference odom
    if (ref_pose_data_buff_.size() == 0) {
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(10ms);
      reference_odom_sub_->parse_data(ref_pose_data_buff_);
      continue;
    }
    // get sync data
    current_ref = ref_pose_data_buff_.front();
    double dt = current_lidar_data_.time - current_ref.time;
    if (dt < -0.05) {
      return false;
    }
    if (dt < -0.05) {
      return false;
    } else if (dt > 0.05) {
      ref_pose_data_buff_.pop_front();
      continue;
    } else {
      odom = current_ref;
      break;
    }
  }
  return true;
}

bool LidarOdometryNode::update()
{
  // set initial pose for lidar_odometry
  if (!inited_) {
    if (use_initial_pose_from_topic_) {
      localization_common::OdomData reference_odom;
      if (!get_sync_reference_odom(reference_odom)) {
        return false;
      }
      initial_pose_ = reference_odom.pose;
      reference_odom_sub_.reset();
    }
    lidar_odometry_->set_initial_pose(initial_pose_);
    inited_ = true;
    RCLCPP_INFO(
      node_->get_logger(), "Lidar Odometry Inited at time [%lf] with Pose (%lf, %lf, %lf) ",
      current_lidar_data_.time, initial_pose_(0, 3), initial_pose_(1, 3), initial_pose_(2, 3));
  }
  // update lidar odometry
  return lidar_odometry_->update(current_lidar_data_);
}

bool LidarOdometryNode::publish_data()
{
  Eigen::Matrix4d current_pose = lidar_odometry_->get_current_pose();
  lidar_odom_pub_->publish(current_pose, current_lidar_data_.time);
  if (publish_tf_) {
    // publish base_link_to_map tf
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = localization_common::to_ros_time(current_lidar_data_.time);
    msg.header.frame_id = "map";
    msg.child_frame_id = base_frame_id_;
    msg.transform = localization_common::to_transform_msg(current_pose);
    tf_pub_->sendTransform(msg);
  }
  if (cloud_pub_->has_subscribers()) {
    cloud_pub_->publish(lidar_odometry_->get_current_scan());
  }
  if (lidar_odometry_->has_new_local_map() && local_map_pub_->has_subscribers()) {
    local_map_pub_->publish(lidar_odometry_->get_local_map());
  }
  return true;
}

}  // namespace lidar_odometry
