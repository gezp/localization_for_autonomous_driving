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

#include "lidar_mapping/back_end_node.hpp"

#include <filesystem>
#include "localization_common/tf_utils.hpp"

namespace lidar_mapping
{
BackEndNode::BackEndNode(rclcpp::Node::SharedPtr node)
{
  std::string back_end_config;
  std::string data_path;
  node->declare_parameter("back_end_config", back_end_config);
  node->declare_parameter("data_path", data_path);
  node->declare_parameter("publish_tf", publish_tf_);
  node->get_parameter("back_end_config", back_end_config);
  node->get_parameter("data_path", data_path);
  node->get_parameter("publish_tf", publish_tf_);
  RCLCPP_INFO(node->get_logger(), "back_end_config: [%s]", back_end_config.c_str());
  RCLCPP_INFO(node->get_logger(), "data_path: [%s]", data_path.c_str());
  if (back_end_config == "" || (!std::filesystem::exists(back_end_config))) {
    RCLCPP_FATAL(node->get_logger(), "back_end_config is invalid");
    return;
  }
  if (data_path == "" || (!std::filesystem::is_directory(data_path))) {
    RCLCPP_FATAL(node->get_logger(), "data_path is invalid");
    return;
  }
  trajectory_path_ = data_path + "/mapping_trajectory";
  //
  std::cout << "-----------------Init Backend-------------------" << std::endl;
  back_end_ = std::make_shared<BackEnd>();
  back_end_->init_config(back_end_config, data_path);
  // sub & pub
  cloud_sub_ = std::make_shared<localization_common::CloudSubscriber>(node, "synced_cloud", 100000);
  gnss_pose_sub_ =
    std::make_shared<localization_common::OdometrySubscriber>(node, "synced_gnss/pose", 100000);
  lidar_odom_sub_ =
    std::make_shared<localization_common::OdometrySubscriber>(node, "lidar_odom", 100000);
  loop_pose_sub_ =
    std::make_shared<localization_common::LoopPoseSubscriber>(node, "loop_pose", 100000);
  key_scan_pub_ =
    std::make_shared<localization_common::CloudPublisher>(node, "key_scan", "lidar", 100);
  key_frame_pub_ =
    std::make_shared<localization_common::KeyFramePublisher>(node, "key_frame", "map", 100);
  key_gnss_pub_ =
    std::make_shared<localization_common::KeyFramePublisher>(node, "key_gnss", "map", 100);
  key_frames_pub_ = std::make_shared<localization_common::KeyFramesPublisher>(
    node, "optimized_path", "map", 100);
  optimized_odom_pub_ = std::make_shared<localization_common::OdometryPublisher>(
    node, "optimized_pose", "map", base_link_frame_id_, 100);
  global_map_pub_ =
    std::make_shared<localization_common::CloudPublisher>(node, "global_map", "map", 100);
  tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  // srv
  optimize_map_srv_ = node->create_service<localization_interfaces::srv::OptimizeMap>(
    "optimize_map",
    [&](
      const localization_interfaces::srv::OptimizeMap::Request::SharedPtr /*request*/,
      const localization_interfaces::srv::OptimizeMap::Response::SharedPtr response) {
      need_optimize_map_ = true;
      response->succeed = true;
    });
  save_map_srv_ = node->create_service<localization_interfaces::srv::SaveMap>(
    "save_map", [this](
      const localization_interfaces::srv::SaveMap::Request::SharedPtr /*request*/,
      localization_interfaces::srv::SaveMap::Response::SharedPtr response) {
      save_map_flag_ = true;
      response->succeed = true;
    });
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

BackEndNode::~BackEndNode()
{
  exit_ = true;
  if (run_thread_) {
    run_thread_->join();
  }
}

bool BackEndNode::run()
{
  if (need_optimize_map_) {
    force_optimize();
    need_optimize_map_ = false;
  }
  if (save_map_flag_) {
    auto optimized_key_frames = back_end_->get_optimized_key_frames();
    save_trajectory(optimized_key_frames);
    back_end_->save_map();
    save_map_flag_ = false;
  }
  // load messages into buffer:
  read_data();
  // add loop poses for graph optimization:
  maybe_insert_loop_pose();
  while (has_data()) {
    if (!valid_data()) {
      continue;
    }
    update_back_end();
    publish_data();
  }
  return true;
}

bool BackEndNode::force_optimize()
{
  back_end_->optimize();
  if (back_end_->has_new_optimized()) {
    auto optimized_key_frames = back_end_->get_optimized_key_frames();
    key_frames_pub_->publish(optimized_key_frames);
  }
  return true;
}

bool BackEndNode::read_data()
{
  cloud_sub_->parse_data(cloud_data_buff_);
  gnss_pose_sub_->parse_data(gnss_pose_data_buff_);
  lidar_odom_sub_->parse_data(lidar_odom_data_buff_);
  loop_pose_sub_->parse_data(loop_pose_data_buff_);
  return true;
}

bool BackEndNode::maybe_insert_loop_pose()
{
  while (loop_pose_data_buff_.size() > 0) {
    back_end_->insert_loop_pose(loop_pose_data_buff_.front());
    loop_pose_data_buff_.pop_front();
  }
  return true;
}

bool BackEndNode::has_data()
{
  if (cloud_data_buff_.size() == 0) {
    return false;
  }
  if (gnss_pose_data_buff_.size() == 0) {
    return false;
  }
  if (lidar_odom_data_buff_.size() == 0) {
    return false;
  }

  return true;
}

bool BackEndNode::valid_data()
{
  current_cloud_data_ = cloud_data_buff_.front();
  current_gnss_pose_data_ = gnss_pose_data_buff_.front();
  current_lidar_odom_data_ = lidar_odom_data_buff_.front();

  double diff_gnss_time = current_cloud_data_.time - current_gnss_pose_data_.time;
  double diff_laser_time = current_cloud_data_.time - current_lidar_odom_data_.time;

  if (diff_gnss_time < -0.05 || diff_laser_time < -0.05) {
    cloud_data_buff_.pop_front();
    return false;
  }

  if (diff_gnss_time > 0.05) {
    gnss_pose_data_buff_.pop_front();
    return false;
  }

  if (diff_laser_time > 0.05) {
    lidar_odom_data_buff_.pop_front();
    return false;
  }
  cloud_data_buff_.pop_front();
  gnss_pose_data_buff_.pop_front();
  lidar_odom_data_buff_.pop_front();

  return true;
}

bool BackEndNode::update_back_end()
{
  // optimization is carried out in map frame:
  back_end_->update(current_cloud_data_, current_lidar_odom_data_, current_gnss_pose_data_);
  if (back_end_->has_new_key_frame()) {
    // write GNSS/IMU pose and lidar odometry estimation as trajectory for evo evaluation:
    trajectory_.time.push_back(current_cloud_data_.time);
    trajectory_.ref.push_back(current_gnss_pose_data_.pose);
    trajectory_.lidar.push_back(current_lidar_odom_data_.pose);
    trajectory_.length++;
  }
  return true;
}

bool BackEndNode::publish_data()
{
  // publish optimized pose
  Eigen::Matrix4f optimized_pose =
    back_end_->get_map_to_lidar_odom() * current_lidar_odom_data_.pose;
  optimized_odom_pub_->publish(optimized_pose, current_lidar_odom_data_.time);
  if (publish_tf_) {
    // publish optimized pose tf
    auto msg =
      localization_common::to_transform_stamped_msg(optimized_pose, current_lidar_odom_data_.time);
    msg.header.frame_id = "map";
    msg.child_frame_id = base_link_frame_id_;
    tf_pub_->sendTransform(msg);
  }
  // publish new key frame
  if (back_end_->has_new_key_frame()) {
    // publish key frame & gnss for loop closure
    localization_common::CloudData key_scan;
    back_end_->get_latest_key_scan(key_scan);
    key_scan_pub_->publish(key_scan.cloud, key_scan.time);
    localization_common::KeyFrame key_frame;
    back_end_->get_latest_key_frame(key_frame);
    key_frame_pub_->publish(key_frame);
    back_end_->get_latest_key_gnss(key_frame);
    key_gnss_pub_->publish(key_frame);
    // publish optimized key frames
    auto optimized_key_frames = back_end_->get_optimized_key_frames();
    key_frames_pub_->publish(optimized_key_frames);
    // publish global map
    if (back_end_->has_new_optimized() && global_map_pub_->has_subscribers()) {
      global_map_pub_->publish(back_end_->get_global_map());
    }
  }
  return true;
}

void BackEndNode::save_pose(std::ofstream & ofs, const Eigen::Matrix4f & pose)
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
}

bool BackEndNode::save_trajectory(
  const std::deque<localization_common::KeyFrame> & optimized_key_frames)
{
  if (std::filesystem::is_directory(trajectory_path_)) {
    std::filesystem::remove_all(trajectory_path_);
  }
  if (!std::filesystem::create_directory(trajectory_path_)) {
    return false;
  }
  std::ofstream ground_truth_ofs, lidar_odom_ofs, optimized_pose_ofs;
  // open files
  ground_truth_ofs.open(trajectory_path_ + "/ground_truth.txt", std::ios::app);
  if (!ground_truth_ofs) {
    return false;
  }
  lidar_odom_ofs.open(trajectory_path_ + "/lidar_odom.txt", std::ios::app);
  if (!lidar_odom_ofs) {
    return false;
  }
  optimized_pose_ofs.open(trajectory_path_ + "/optimized.txt", std::ios::app);
  if (!optimized_pose_ofs) {
    return false;
  }
  // save to file
  assert(trajectory_.length == optimized_key_frames.size());
  for (size_t i = 0; i < trajectory_.length; ++i) {
    save_pose(ground_truth_ofs, trajectory_.ref[i]);
    save_pose(lidar_odom_ofs, trajectory_.lidar[i]);
    save_pose(optimized_pose_ofs, optimized_key_frames[i].pose);
  }
  return true;
}

}  // namespace lidar_mapping
