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

namespace graph_based_localization
{

SlidingWindowNode::SlidingWindowNode(rclcpp::Node::SharedPtr node)
{
  std::string config_file;
  node->declare_parameter("config_file", config_file);
  node->get_parameter("config_file", config_file);
  std::cout << "config file path:" << config_file << std::endl;
  // sub&pub
  lidar_pose_sub_ = std::make_shared<localization_common::OdometrySubscriber>(
    node, "localization/lidar/pose", 10000);
  gnss_pose_sub_ =
    std::make_shared<localization_common::OdometrySubscriber>(node, "synced_gnss/pose", 10000);
  imu_raw_sub_ =
    std::make_shared<localization_common::ImuSubscriber>(node, "/kitti/oxts/imu/extract", 100000);
  imu_synced_sub_ = std::make_shared<localization_common::ImuSubscriber>(node, "synced_imu", 10000);
  optimized_odom_pub_ = std::make_shared<localization_common::OdometryPublisher>(
    node, "localization/fused/pose", "map", "base_link", 100);
  // tf:
  imu_frame_id_ = "imu_link";
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
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
          std::this_thread::sleep_for(10ms);
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

bool SlidingWindowNode::init_calibration()
{
  // lookup imu pose in lidar frame:
  static bool calibration_received = false;
  if (!calibration_received) {
    if (localization_common::lookup_in_tf_buffer(
        tf_buffer_, imu_frame_id_, base_link_frame_id_, base_link_to_imu_))
    {
      sliding_window_->set_extrinsic(base_link_to_imu_.inverse());
      calibration_received = true;
    }
  }
  return calibration_received;
}

bool SlidingWindowNode::run()
{
  if (!init_calibration()) {
    return false;
  }
  if (!read_data()) {
    return false;
  }
  while (has_data()) {
    if (!valid_data()) {
      continue;
    }
    update_back_end();
    publish_data();
  }
  return true;
}

bool SlidingWindowNode::read_data()
{
  lidar_pose_sub_->parse_data(lidar_pose_data_buff_);
  imu_raw_sub_->parse_data(imu_raw_data_buff_);
  imu_synced_sub_->parse_data(imu_synced_data_buff_);
  gnss_pose_sub_->parse_data(gnss_pose_data_buff_);
  return true;
}

bool SlidingWindowNode::has_data()
{
  if (
    lidar_pose_data_buff_.empty() || imu_synced_data_buff_.empty() ||
    gnss_pose_data_buff_.empty())
  {
    return false;
  }
  return true;
}

bool SlidingWindowNode::valid_data()
{
  current_lidar_pose_data_ = lidar_pose_data_buff_.front();
  current_imu_data_ = imu_synced_data_buff_.front();
  current_gnss_pose_data_ = gnss_pose_data_buff_.front();

  double diff_imu_time = current_lidar_pose_data_.time - current_imu_data_.time;
  double diff_gnss_pose_time = current_lidar_pose_data_.time - current_gnss_pose_data_.time;

  if (diff_imu_time < -0.05 || diff_gnss_pose_time < -0.05) {
    lidar_pose_data_buff_.pop_front();
    return false;
  }

  if (diff_imu_time > 0.05) {
    imu_synced_data_buff_.pop_front();
    return false;
  }

  if (diff_gnss_pose_time > 0.05) {
    gnss_pose_data_buff_.pop_front();
    return false;
  }

  lidar_pose_data_buff_.pop_front();
  imu_synced_data_buff_.pop_front();
  gnss_pose_data_buff_.pop_front();

  return true;
}

bool SlidingWindowNode::update_back_end()
{
  // update IMU pre-integration:
  while (!imu_raw_data_buff_.empty() && imu_raw_data_buff_.front().time < current_imu_data_.time) {
    sliding_window_->add_raw_imu(imu_raw_data_buff_.front());
    imu_raw_data_buff_.pop_front();
  }
  // optimization is carried out in map frame:
  return sliding_window_->update(
    current_lidar_pose_data_, current_imu_data_,
    current_gnss_pose_data_);
}

bool SlidingWindowNode::publish_data()
{
  if (!sliding_window_->has_new_optimized()) {
    return false;
  }
  // fused_pose in map frame, fused_vel in imu frame
  // TODO(gezp) : move fused_vel to base_link frame.
  auto nav_state = sliding_window_->get_imu_nav_state();
  std::cout << "ba: " << nav_state.accel_bias.transpose() << ",bg:" <<
    nav_state.gyro_bias.transpose() << std::endl;
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
  optimized_odom_pub_->publish(fused_pose, fused_vel, nav_state.time);
  return true;
}

}  // namespace graph_based_localization
