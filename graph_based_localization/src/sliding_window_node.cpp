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

#include "graph_based_localization/sliding_window_node.hpp"

#include "localization_common/msg_util.hpp"

namespace graph_based_localization
{

SlidingWindowNode::SlidingWindowNode(rclcpp::Node::SharedPtr node)
{
  std::string config_file;
  node->declare_parameter("config_file", config_file);
  node->declare_parameter("base_frame_id", base_frame_id_);
  node->declare_parameter("imu_frame_id", imu_frame_id_);
  node->get_parameter("config_file", config_file);
  node->get_parameter("base_frame_id", base_frame_id_);
  node->get_parameter("imu_frame_id", imu_frame_id_);
  std::cout << "config file path:" << config_file << std::endl;
  // sub&pub
  lidar_pose_sub_ = std::make_shared<localization_common::OdometrySubscriber>(
    node, "localization/lidar/pose", 10000);
  gnss_pose_sub_ =
    std::make_shared<localization_common::OdometrySubscriber>(node, "synced_gnss/pose", 10000);
  raw_imu_sub_ =
    std::make_shared<localization_common::ImuSubscriber>(node, "/kitti/oxts/imu/extract", 100000);
  optimized_odom_pub_ = std::make_shared<localization_common::OdometryPublisher>(
    node, "localization/fused/pose", "map", base_frame_id_, 100);
  // tf:
  tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  // extrinsics
  extrinsics_manager_ = std::make_shared<localization_common::ExtrinsicsManager>(node);
  extrinsics_manager_->enable_tf_listener();
  // sliding_window:
  YAML::Node config_node = YAML::LoadFile(config_file);
  sliding_window_ = std::make_shared<SlidingWindow>();
  sliding_window_->init_with_config(config_node);
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

SlidingWindowNode::~SlidingWindowNode()
{
  exit_ = true;
  if (run_thread_) {
    run_thread_->join();
  }
}

bool SlidingWindowNode::read_data()
{
  lidar_pose_sub_->parse_data(lidar_pose_buffer_);
  raw_imu_sub_->parse_data(raw_imu_data_buffer_);
  gnss_pose_sub_->parse_data(gnss_pose_buffer_);
  return true;
}

bool SlidingWindowNode::has_data()
{
  if (lidar_pose_buffer_.empty() || gnss_pose_buffer_.empty()) {
    return false;
  }
  return true;
}

bool SlidingWindowNode::valid_data()
{
  current_lidar_pose_ = lidar_pose_buffer_.front();
  current_gnss_pose_ = gnss_pose_buffer_.front();

  double diff_gnss_pose_time = current_lidar_pose_.time - current_gnss_pose_.time;

  if (diff_gnss_pose_time < -0.05) {
    lidar_pose_buffer_.pop_front();
    return false;
  }

  if (diff_gnss_pose_time > 0.05) {
    gnss_pose_buffer_.pop_front();
    return false;
  }

  lidar_pose_buffer_.pop_front();
  gnss_pose_buffer_.pop_front();
  return true;
}

bool SlidingWindowNode::run()
{
  // get extrinsics
  if (!is_valid_extrinsics_) {
    if (!extrinsics_manager_->lookup(base_frame_id_, imu_frame_id_, T_base_imu_)) {
      return false;
    }
    sliding_window_->set_extrinsic(T_base_imu_);
    is_valid_extrinsics_ = true;
  }
  // read data
  read_data();
  // process data
  if (!raw_imu_data_buffer_.empty()) {
    sliding_window_->add_imu_data(raw_imu_data_buffer_.front());
    raw_imu_data_buffer_.pop_front();
  }
  if (has_data() && valid_data()) {
    sliding_window_->add_gnss_pose(current_gnss_pose_);
    sliding_window_->add_lidar_pose(current_lidar_pose_);
  }
  if (sliding_window_->update()) {
    publish_data();
    return true;
  }
  return false;
}

bool SlidingWindowNode::publish_data()
{
  if (sliding_window_->has_new_optimized()) {
    // get ba and bg
    auto nav_state = sliding_window_->get_imu_nav_state();
    std::cout << "ba: " << nav_state.accel_bias.transpose()
              << ",bg:" << nav_state.gyro_bias.transpose() << std::endl;
  }
  // get odom
  auto odom = sliding_window_->get_current_odom();
  // publish tf
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = localization_common::to_ros_time(odom.time);
  msg.header.frame_id = "map";
  msg.child_frame_id = base_frame_id_;
  msg.transform = localization_common::to_transform_msg(odom.pose);
  tf_pub_->sendTransform(msg);
  // publish fusion odometry
  optimized_odom_pub_->publish(odom);
  return true;
}

}  // namespace graph_based_localization
