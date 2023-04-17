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

#include "lidar_localization/matching_node.hpp"

#include <filesystem>
#include "localization_common/tf_utils.hpp"

namespace lidar_localization
{
MatchingNode::MatchingNode(rclcpp::Node::SharedPtr node)
{
  std::string matching_config;
  std::string data_path;
  node->declare_parameter("matching_config", matching_config);
  node->declare_parameter("data_path", data_path);
  node->declare_parameter("publish_tf", publish_tf_);
  node->get_parameter("matching_config", matching_config);
  node->get_parameter("data_path", data_path);
  node->get_parameter("publish_tf", publish_tf_);
  RCLCPP_INFO(node->get_logger(), "matching_config: [%s]", matching_config.c_str());
  RCLCPP_INFO(node->get_logger(), "data_path: [%s]", data_path.c_str());
  if (matching_config == "" || (!std::filesystem::exists(matching_config))) {
    RCLCPP_FATAL(node->get_logger(), "matching_config is invalid");
    return;
  }
  if (data_path == "" || (!std::filesystem::is_directory(data_path))) {
    RCLCPP_FATAL(node->get_logger(), "data_path is invalid");
    return;
  }
  // subscriber:
  cloud_sub_ = std::make_shared<localization_common::CloudSubscriber>(node, "synced_cloud", 10000);
  gnss_sub_ =
    std::make_shared<localization_common::OdometrySubscriber>(node, "synced_gnss/pose", 10000);
  // publisher:
  global_map_pub_ = std::make_shared<localization_common::CloudPublisher>(
    node, "lidar_localization/global_map", "map", 100);
  local_map_pub_ = std::make_shared<localization_common::CloudPublisher>(
    node, "lidar_localization/local_map", "map", 100);
  current_scan_pub_ = std::make_shared<localization_common::CloudPublisher>(
    node, "lidar_localization/current_scan", "map", 100);
  lidar_odom_pub_ = std::make_shared<localization_common::OdometryPublisher>(
    node, "localization/lidar/pose", "map", base_link_frame_id_, 100);
  tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  std::cout << "-----------------Init Matching-------------------" << std::endl;
  matching_ = std::make_shared<Matching>();
  matching_->init_config(matching_config, data_path);
  // process loop flow callback
  run_thread_ = std::make_unique<std::thread>(
    [this]() {
      while (!exit_) {
        run();
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(10ms);
      }
    });
}

MatchingNode::~MatchingNode()
{
  exit_ = true;
  if (run_thread_) {
    run_thread_->join();
  }
}

bool MatchingNode::run()
{
  read_data();
  while (has_data()) {
    if (!valid_data()) {
      std::cout << "Invalid data. Skip matching" << std::endl;
      continue;
    }
    if (update_matching()) {
      publish_data();
    }
  }

  static bool global_map_published = false;
  if (!global_map_published && global_map_pub_->has_subscribers()) {
    auto global_map = matching_->get_global_map();
    global_map_pub_->publish(global_map);
    global_map_published = true;
  }

  if (matching_->has_new_local_map() && local_map_pub_->has_subscribers()) {
    local_map_pub_->publish(matching_->get_local_map());
  }
  return true;
}

bool MatchingNode::read_data()
{
  // pipe lidar measurements and pose into buffer:
  cloud_sub_->parse_data(cloud_data_buff_);
  gnss_sub_->parse_data(gnss_data_buff_);
  return true;
}

bool MatchingNode::has_data()
{
  if (cloud_data_buff_.size() == 0) {
    return false;
  }
  if (gnss_data_buff_.size() == 0) {
    return false;
  }
  return true;
}

bool MatchingNode::valid_data()
{
  current_cloud_data_ = cloud_data_buff_.front();
  current_gnss_data_ = gnss_data_buff_.front();
  double diff_time = current_cloud_data_.time - current_gnss_data_.time;
  if (diff_time < -0.05) {
    cloud_data_buff_.pop_front();
    return false;
  }
  if (diff_time > 0.05) {
    gnss_data_buff_.pop_front();
    return false;
  }
  cloud_data_buff_.pop_front();
  gnss_data_buff_.pop_front();
  return true;
}

bool MatchingNode::update_matching()
{
  if (!matching_->has_inited()) {
    // global initialization
    if (matching_->set_init_pose_by_scan_context(current_cloud_data_)) {
      Eigen::Matrix4f init_pose = matching_->get_init_pose();
      // evaluate deviation from GNSS/IMU:
      float deviation =
        (init_pose.block<3, 1>(0, 3) - current_gnss_data_.pose.block<3, 1>(0, 3)).norm();
      std::cout << "Scan Context Localization Init Succeeded. Deviation between GNSS/IMU: "
                << deviation << std::endl;
    } else {
      // if failed, fall back to GNSS/IMU init:
      matching_->set_init_pose_by_gnss(current_gnss_data_.pose);
      std::cout << "Scan Context Localization Init Failed. Fallback to GNSS/IMU." << std::endl;
    }
  }
  return matching_->update(current_cloud_data_, lidar_odometry_);
}

bool MatchingNode::publish_data()
{
  lidar_odom_pub_->publish(lidar_odometry_, current_cloud_data_.time);
  if (publish_tf_) {
    auto msg =
      localization_common::to_transform_stamped_msg(lidar_odometry_, current_cloud_data_.time);
    msg.header.frame_id = "map";
    msg.child_frame_id = base_link_frame_id_;
    tf_pub_->sendTransform(msg);
  }
  if (current_scan_pub_->has_subscribers()) {
    current_scan_pub_->publish(matching_->get_current_scan());
  }
  return true;
}

}  // namespace lidar_localization
