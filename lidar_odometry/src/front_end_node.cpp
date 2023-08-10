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

#include "lidar_odometry/front_end_node.hpp"

#include <filesystem>

namespace lidar_odometry
{
FrontEndNode::FrontEndNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  // front_end_config
  std::string front_end_config;
  node->declare_parameter("front_end_config", front_end_config);
  node->declare_parameter("publish_tf", publish_tf_);
  node->declare_parameter("use_init_pose_from_gnss", use_init_pose_from_gnss_);
  node->get_parameter("front_end_config", front_end_config);
  node->get_parameter("publish_tf", publish_tf_);
  node->get_parameter("use_init_pose_from_gnss", use_init_pose_from_gnss_);
  RCLCPP_INFO(node->get_logger(), "front_end_config: [%s]", front_end_config.c_str());
  if (front_end_config == "" || (!std::filesystem::exists(front_end_config))) {
    RCLCPP_FATAL(node->get_logger(), "front_end_config is invalid");
    return;
  }
  // init front end
  front_end_ = std::make_shared<FrontEnd>();
  front_end_->init_config(front_end_config);
  // init sub & pub:
  cloud_sub_ = std::make_shared<localization_common::CloudSubscriber<pcl::PointXYZ>>(
    node, "synced_cloud", 10000);
  gnss_sub_ =
    std::make_shared<localization_common::OdometrySubscriber>(node, "synced_gnss/pose", 10000);
  cloud_pub_ = std::make_shared<localization_common::CloudPublisher<pcl::PointXYZ>>(
    node, "lidar_odometry/current_scan", "map", 100);
  local_map_pub_ = std::make_shared<localization_common::CloudPublisher<pcl::PointXYZ>>(
    node, "lidar_odometry/local_map", "map", 100);
  lidar_odom_pub_ = std::make_shared<localization_common::OdometryPublisher>(
    node, "lidar_odom", "map", base_link_frame_id_, 100);
  tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
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

FrontEndNode::~FrontEndNode()
{
  exit_ = true;
  if (run_thread_) {
    run_thread_->join();
  }
}

bool FrontEndNode::run()
{
  if (!read_data()) {
    return false;
  }
  while (has_data()) {
    if (!valid_data()) {
      continue;
    }
    if (update_odometry()) {
      publish_data();
    } else {
      RCLCPP_WARN(node_->get_logger(), "Update Odometry failed!");
    }
  }
  return true;
}

bool FrontEndNode::read_data()
{
  cloud_sub_->parse_data(lidar_data_buff_);
  gnss_sub_->parse_data(gnss_pose_data_buff_);
  return true;
}

bool FrontEndNode::has_data()
{
  if (lidar_data_buff_.size() == 0) {
    return false;
  }
  if (gnss_pose_data_buff_.size() == 0) {
    return false;
  }
  return true;
}

bool FrontEndNode::valid_data()
{
  current_lidar_data_ = lidar_data_buff_.front();
  current_gnss_pose_data_ = gnss_pose_data_buff_.front();

  double d_time = current_lidar_data_.time - current_gnss_pose_data_.time;
  if (d_time < -0.05) {
    lidar_data_buff_.pop_front();
    return false;
  }

  if (d_time > 0.05) {
    gnss_pose_data_buff_.pop_front();
    return false;
  }

  lidar_data_buff_.pop_front();
  gnss_pose_data_buff_.pop_front();

  return true;
}

bool FrontEndNode::update_odometry()
{
  // set init pose for lidar_odometry
  if (!odometry_inited_) {
    if (use_init_pose_from_gnss_) {
      front_end_->set_init_pose(current_gnss_pose_data_.pose);
    } else {
      front_end_->set_init_pose(Eigen::Matrix4d::Identity());
    }
    odometry_inited_ = true;
  }
  // update lidar_odometry
  return front_end_->update(current_lidar_data_);
}

bool FrontEndNode::publish_data()
{
  Eigen::Matrix4d current_pose = front_end_->get_current_pose();
  lidar_odom_pub_->publish(current_pose, current_lidar_data_.time);
  if (publish_tf_) {
    // publish base_link_to_map tf
    auto msg = localization_common::to_transform_stamped_msg(
      current_pose.cast<float>(), current_lidar_data_.time);
    msg.header.frame_id = "map";
    msg.child_frame_id = base_link_frame_id_;
    tf_pub_->sendTransform(msg);
  }
  if (cloud_pub_->has_subscribers()) {
    cloud_pub_->publish(front_end_->get_current_scan());
  }
  if (front_end_->has_new_local_map() && local_map_pub_->has_subscribers()) {
    local_map_pub_->publish(front_end_->get_local_map());
  }
  return true;
}

}  // namespace lidar_odometry
