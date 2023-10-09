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

#include "localization_common/sensor_data_utils.hpp"

namespace lidar_odometry
{
LidarOdometryNode::LidarOdometryNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  // lidar_odometry_config
  std::string lidar_odometry_config;
  node->declare_parameter("lidar_odometry_config", lidar_odometry_config);
  node->declare_parameter("publish_tf", publish_tf_);
  node->declare_parameter("use_initial_pose_from_topic", use_initial_pose_from_topic_);
  node->declare_parameter("base_frame_id", base_frame_id_);
  node->declare_parameter("lidar_frame_id", lidar_frame_id_);
  node->get_parameter("lidar_odometry_config", lidar_odometry_config);
  node->get_parameter("publish_tf", publish_tf_);
  node->get_parameter("use_initial_pose_from_topic", use_initial_pose_from_topic_);
  node->get_parameter("base_frame_id", base_frame_id_);
  node->get_parameter("lidar_frame_id", lidar_frame_id_);
  RCLCPP_INFO(node->get_logger(), "lidar_odometry_config: [%s]", lidar_odometry_config.c_str());
  if (lidar_odometry_config == "" || (!std::filesystem::exists(lidar_odometry_config))) {
    RCLCPP_FATAL(node->get_logger(), "lidar_odometry_config is invalid");
    return;
  }
  // lidar_odometry
  lidar_odometry_ = std::make_shared<LidarOdometry>();
  lidar_odometry_->init_config(lidar_odometry_config);
  // sub & pub
  cloud_sub_ = std::make_shared<localization_common::CloudSubscriber>(
    node, "synced_cloud", 10000);
  if (use_initial_pose_from_topic_) {
    reference_odom_sub_ =
      std::make_shared<localization_common::OdometrySubscriber>(node, "reference_odom", 10000);
  }
  current_scan_pub_ = std::make_shared<localization_common::CloudPublisher>(
    node, "lidar_odometry/current_scan", "map", 100);
  local_map_pub_ = std::make_shared<localization_common::CloudPublisher>(
    node, "lidar_odometry/local_map", "map", 100);
  lidar_odom_pub_ = std::make_shared<localization_common::OdometryPublisher>(
    node, "lidar_odom", "map", base_frame_id_, 100);
  tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  // extrinsics
  extrinsics_manager_ = std::make_shared<localization_common::ExtrinsicsManager>(node);
  extrinsics_manager_->enable_tf_listener();
  // buffer
  ref_odom_buffer_ = std::make_shared<localization_common::OdomDataBuffer>(10000);
  // process loop callback
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
  // read data
  cloud_sub_->parse_data(lidar_data_buffer_);
  // set initial pose for better visualization
  if (use_initial_pose_from_topic_ && !inited_) {
    if (set_initial_pose_by_reference_odom()) {
      inited_ = true;
    }
  }
  // process lidar data
  if (!lidar_data_buffer_.empty()) {
    if (lidar_odometry_->update(lidar_data_buffer_.front())) {
      publish_data();
    }
    lidar_data_buffer_.pop_front();
    return true;
  }
  return false;
}

bool LidarOdometryNode::set_initial_pose_by_reference_odom()
{
  // read reference_odom data
  std::deque<localization_common::OdomData> buffer;
  reference_odom_sub_->parse_data(buffer);
  for (auto & data : buffer) {
    ref_odom_buffer_->add_data(data);
  }
  // check
  if (lidar_data_buffer_.empty() || ref_odom_buffer_->get_size() == 0) {
    return false;
  }
  if (lidar_data_buffer_.front().time < ref_odom_buffer_->get_start_time()) {
    lidar_data_buffer_.pop_front();
    std::cout << "drop earlier lidar data" << std::endl;
    return false;
  }
  if (lidar_data_buffer_.front().time > ref_odom_buffer_->get_end_time()) {
    // wait valid ref odom
    return false;
  }
  // get synced odom
  localization_common::OdomData odom;
  if (!ref_odom_buffer_->get_interpolated_data(lidar_data_buffer_.front().time, odom)) {
    return false;
  }
  // set initial pose
  lidar_odometry_->set_initial_pose(odom.pose);
  Eigen::Vector3d p = odom.pose.block<3, 1>(0, 3);
  RCLCPP_INFO(node_->get_logger(), "initialize at position: (%lf, %lf, %lf)", p.x(), p.y(), p.z());
  reference_odom_sub_.reset();
  return true;
}

void LidarOdometryNode::publish_data()
{
  // publish odom
  auto odom = lidar_odometry_->get_current_odom();
  lidar_odom_pub_->publish(odom);
  // publish tf
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = localization_common::to_ros_time(odom.time);
    msg.header.frame_id = "map";
    msg.child_frame_id = base_frame_id_;
    msg.transform = localization_common::to_transform_msg(odom.pose);
    tf_pub_->sendTransform(msg);
  }
  // publish point cloud
  if (current_scan_pub_->has_subscribers()) {
    current_scan_pub_->publish(*lidar_odometry_->get_current_scan());
  }
  if (lidar_odometry_->has_new_local_map() && local_map_pub_->has_subscribers()) {
    local_map_pub_->publish(*lidar_odometry_->get_local_map());
  }
}

}  // namespace lidar_odometry
