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
  // data_path and front_end_config
  std::string front_end_config;
  node->declare_parameter("data_path", data_path_);
  node->declare_parameter("front_end_config", front_end_config);
  node->declare_parameter("publish_tf", publish_tf_);
  node->get_parameter("data_path", data_path_);
  node->get_parameter("front_end_config", front_end_config);
  node->get_parameter("publish_tf", publish_tf_);
  RCLCPP_INFO(node->get_logger(), "data_path: [%s]", data_path_.c_str());
  RCLCPP_INFO(node->get_logger(), "front_end_config: [%s]", front_end_config.c_str());
  if (data_path_ == "" || (!std::filesystem::is_directory(data_path_))) {
    RCLCPP_FATAL(node->get_logger(), "data_path is invalid");
    return;
  }
  if (front_end_config == "" || (!std::filesystem::exists(front_end_config))) {
    RCLCPP_FATAL(node->get_logger(), "front_end_config is invalid");
    return;
  }
  // init front end
  front_end_ = std::make_shared<FrontEnd>();
  front_end_->init_config(front_end_config);
  // init sub & pub:
  cloud_sub_ = std::make_shared<localization_common::CloudSubscriber>(node, "synced_cloud", 10000);
  gnss_sub_ =
    std::make_shared<localization_common::OdometrySubscriber>(node, "synced_gnss/pose", 10000);
  cloud_pub_ = std::make_shared<localization_common::CloudPublisher>(
    node, "lidar_odometry/current_scan", "map", 100);
  local_map_pub_ = std::make_shared<localization_common::CloudPublisher>(
    node, "lidar_odometry/local_map", "map", 100);
  lidar_odom_pub_ = std::make_shared<localization_common::OdometryPublisher>(
    node, "lidar_odom", "map", base_link_frame_id_, 100);
  tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  // save map callback
  save_odometry_srv_ = node->create_service<localization_interfaces::srv::SaveOdometry>(
    "save_odometry",
    [this](
      const localization_interfaces::srv::SaveOdometry::Request::SharedPtr /*request*/,
      localization_interfaces::srv::SaveOdometry::Response::SharedPtr response) {
      save_odometry_flag_ = true;
      response->succeed = true;
    });
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
  if (save_odometry_flag_) {
    // save trajectory
    save_trajectory();
    save_odometry_flag_ = false;
  }
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
      RCLCPP_WARN(node_->get_logger(), "UpdateLaserOdometry failed!");
    }
  }
  return true;
}

bool FrontEndNode::read_data()
{
  cloud_sub_->parse_data(cloud_data_buff_);
  gnss_sub_->parse_data(gnss_pose_data_buff_);
  return true;
}

bool FrontEndNode::has_data()
{
  if (cloud_data_buff_.size() == 0) {
    return false;
  }
  if (gnss_pose_data_buff_.size() == 0) {
    return false;
  }
  return true;
}

bool FrontEndNode::valid_data()
{
  current_cloud_data_ = cloud_data_buff_.front();
  current_gnss_pose_data_ = gnss_pose_data_buff_.front();

  double d_time = current_cloud_data_.time - current_gnss_pose_data_.time;
  if (d_time < -0.05) {
    cloud_data_buff_.pop_front();
    return false;
  }

  if (d_time > 0.05) {
    gnss_pose_data_buff_.pop_front();
    return false;
  }

  cloud_data_buff_.pop_front();
  gnss_pose_data_buff_.pop_front();

  return true;
}

bool FrontEndNode::update_odometry()
{
  // set init pose for lidar_odometry
  static bool front_end_pose_inited = false;
  if (!front_end_pose_inited) {
    front_end_pose_inited = true;
    front_end_->set_init_pose(current_gnss_pose_data_.pose);
  }

  // update lidar_odometry
  lidar_odom_pose_ = Eigen::Matrix4f::Identity();
  if (front_end_->update(current_cloud_data_, lidar_odom_pose_)) {
    trajectory_.time.push_back(current_gnss_pose_data_.time);
    trajectory_.ref.push_back(current_gnss_pose_data_.pose);
    trajectory_.lidar.push_back(lidar_odom_pose_);
    trajectory_.length++;
    return true;
  }
  return false;
}

bool FrontEndNode::publish_data()
{
  lidar_odom_pub_->publish(lidar_odom_pose_);
  if (publish_tf_) {
    // publish base_link_to_map tf
    auto msg =
      localization_common::to_transform_stamped_msg(lidar_odom_pose_, current_cloud_data_.time);
    msg.header.frame_id = "map";
    msg.child_frame_id = base_link_frame_id_;
    tf_pub_->sendTransform(msg);
  }
  if (cloud_pub_->has_subscribers()) {
    auto current_scan = front_end_->get_current_scan();
    cloud_pub_->publish(current_scan);
  }
  if (front_end_->has_new_local_map() && local_map_pub_->has_subscribers()) {
    auto local_map = front_end_->get_local_map();
    local_map_pub_->publish(local_map);
  }
  return true;
}

bool FrontEndNode::save_pose(std::ofstream & ofs, const Eigen::Matrix4f & pose)
{
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      ofs << pose(i, j);

      if (i == 2 && j == 3) {
        ofs << std::endl;
      } else {
        ofs << " ";
      }
    }
  }
  return true;
}

bool FrontEndNode::save_trajectory()
{
  auto trajectory_dir_path = data_path_ + "/trajectory";
  // remove previous trajectory dir
  if (std::filesystem::is_directory(trajectory_dir_path)) {
    std::filesystem::remove_all(trajectory_dir_path);
  }
  // create trajectory dir
  if (!std::filesystem::create_directory(trajectory_dir_path)) {
    return false;
  }
  std::ofstream ground_truth, lidar_odom;
  // open trajectory files
  ground_truth.open(trajectory_dir_path + "/ground_truth.txt", std::ios::app);
  if (!ground_truth) {
    return false;
  }
  lidar_odom.open(trajectory_dir_path + "/lidar_odom.txt", std::ios::app);
  if (!lidar_odom) {
    return false;
  }
  // save
  for (size_t i = 0; i < trajectory_.length; i++) {
    save_pose(ground_truth, trajectory_.ref[i]);
    save_pose(lidar_odom, trajectory_.lidar[i]);
  }
  RCLCPP_INFO(
    node_->get_logger(), "save trajectory successfully. trajectory dir: [%s]",
    trajectory_dir_path.c_str());
  return true;
}

}  // namespace lidar_odometry
