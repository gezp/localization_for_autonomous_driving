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

#include "loosely_lio_mapping/lio_back_end_node.hpp"

#include <filesystem>
#include "localization_common/tf_utils.hpp"
#include "localization_common/sensor_data_utils.hpp"

namespace loosely_lio_mapping
{
LioBackEndNode::LioBackEndNode(rclcpp::Node::SharedPtr node)
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
  back_end_ = std::make_shared<LioBackEnd>();
  if (!back_end_->init_config(back_end_config, data_path)) {
    std::cout << "faild to init backend!" << std::endl;
    return;
  }
  // init map_generator
  map_generator_ = std::make_shared<lidar_mapping::MapGenerator>();
  if (!map_generator_->init_config(back_end_config, data_path)) {
    std::cout << "faild to init map_generator!" << std::endl;
    return;
  }
  // sub & pub
  cloud_sub_ = std::make_shared<localization_common::CloudSubscriber<pcl::PointXYZ>>(
    node,
    "synced_cloud",
    100000);
  gnss_pose_sub_ =
    std::make_shared<localization_common::OdometrySubscriber>(node, "synced_gnss/pose", 100000);
  lidar_odom_sub_ =
    std::make_shared<localization_common::OdometrySubscriber>(node, "lidar_odom", 100000);
  loop_pose_sub_ =
    std::make_shared<localization_common::LoopPoseSubscriber>(node, "loop_pose", 100000);
  imu_raw_sub_ =
    std::make_shared<localization_common::ImuSubscriber>(node, "/kitti/oxts/imu/extract", 1000000);
  imu_synced_sub_ =
    std::make_shared<localization_common::ImuSubscriber>(node, "synced_imu", 100000);
  key_scan_pub_ =
    std::make_shared<localization_common::CloudPublisher<pcl::PointXYZ>>(
    node, "key_scan", "lidar",
    100);
  key_frame_pub_ =
    std::make_shared<localization_common::KeyFramePublisher>(node, "key_frame", "map", 100);
  key_gnss_pub_ =
    std::make_shared<localization_common::KeyFramePublisher>(node, "key_gnss", "map", 100);
  optimized_path_pub_ =
    std::make_shared<localization_common::PathPublisher>(node, "optimized_path", "map", 100);
  optimized_odom_pub_ = std::make_shared<localization_common::OdometryPublisher>(
    node, "optimized_pose", "map", base_link_frame_id_, 100);
  global_map_pub_ =
    std::make_shared<localization_common::CloudPublisher<pcl::PointXYZ>>(
    node, "global_map", "map",
    100);
  // tf
  imu_frame_id_ = "imu_link";
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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

LioBackEndNode::~LioBackEndNode()
{
  exit_ = true;
  if (run_thread_) {
    run_thread_->join();
  }
}

bool LioBackEndNode::init_calibration()
{
  // lookup imu pose in lidar frame:
  static bool calibration_received = false;
  if (!calibration_received) {
    if (localization_common::lookup_in_tf_buffer(
        tf_buffer_, imu_frame_id_, base_link_frame_id_, base_link_to_imu_))
    {
      back_end_->set_imu_extrinsic(base_link_to_imu_.inverse());
      calibration_received = true;
    }
  }
  return calibration_received;
}

bool LioBackEndNode::run()
{
  if (!init_calibration()) {
    return false;
  }
  if (need_optimize_map_) {
    force_optimize();
    need_optimize_map_ = false;
  }
  if (save_map_flag_) {
    auto optimized_key_frames = back_end_->get_optimized_key_frames();
    save_trajectory(optimized_key_frames);
    map_generator_->save_map(optimized_key_frames);
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

bool LioBackEndNode::force_optimize()
{
  back_end_->optimize();
  if (back_end_->has_new_optimized()) {
    // publish optimized key frames
    auto optimized_key_frames = back_end_->get_optimized_key_frames();
    optimized_path_pub_->publish(optimized_key_frames);
    // publish global map
    if (back_end_->has_new_optimized() && global_map_pub_->has_subscribers()) {
      auto cloud = map_generator_->get_global_map(optimized_key_frames, true);
      global_map_pub_->publish(cloud);
    }
  }
  return true;
}

bool LioBackEndNode::read_data()
{
  cloud_sub_->parse_data(lidar_data_buff_);
  gnss_pose_sub_->parse_data(gnss_pose_data_buff_);
  lidar_odom_sub_->parse_data(lidar_odom_data_buff_);
  loop_pose_sub_->parse_data(loop_pose_data_buff_);
  imu_raw_sub_->parse_data(imu_raw_data_buff_);
  imu_synced_sub_->parse_data(imu_synced_data_buff_);
  return true;
}

bool LioBackEndNode::maybe_insert_loop_pose()
{
  while (loop_pose_data_buff_.size() > 0) {
    back_end_->insert_loop_pose(loop_pose_data_buff_.front());
    loop_pose_data_buff_.pop_front();
  }
  return true;
}

bool LioBackEndNode::has_data()
{
  if (lidar_data_buff_.size() == 0) {
    return false;
  }
  if (gnss_pose_data_buff_.size() == 0) {
    return false;
  }
  if (lidar_odom_data_buff_.size() == 0) {
    return false;
  }
  if (imu_synced_data_buff_.size() == 0) {
    return false;
  }
  return true;
}

bool LioBackEndNode::valid_data()
{
  current_lidar_data_ = lidar_data_buff_.front();
  current_gnss_pose_data_ = gnss_pose_data_buff_.front();
  current_lidar_odom_data_ = lidar_odom_data_buff_.front();
  current_imu_data_ = imu_synced_data_buff_.front();

  double diff_gnss_time = current_lidar_data_.time - current_gnss_pose_data_.time;
  double diff_laser_time = current_lidar_data_.time - current_lidar_odom_data_.time;
  double diff_imu_time = current_lidar_data_.time - current_imu_data_.time;

  if (diff_gnss_time < -0.05 || diff_laser_time < -0.05) {
    lidar_data_buff_.pop_front();
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
  if (diff_imu_time > 0.05) {
    imu_synced_data_buff_.pop_front();
    return false;
  }
  lidar_data_buff_.pop_front();
  gnss_pose_data_buff_.pop_front();
  lidar_odom_data_buff_.pop_front();
  imu_synced_data_buff_.pop_front();
  return true;
}

bool LioBackEndNode::update_imu_pre_integration()
{
  while (!imu_raw_data_buff_.empty() && imu_raw_data_buff_.front().time < current_imu_data_.time) {
    back_end_->add_raw_imu(imu_raw_data_buff_.front());
    imu_raw_data_buff_.pop_front();
  }
  return true;
}

bool LioBackEndNode::update_back_end()
{
  update_imu_pre_integration();
  back_end_->update(
    current_lidar_data_, current_lidar_odom_data_, current_gnss_pose_data_, current_imu_data_);
  if (back_end_->has_new_key_frame()) {
    // write GNSS/IMU pose and lidar odometry estimation as trajectory for evo evaluation:
    trajectory_.time.push_back(current_lidar_data_.time);
    trajectory_.ref.push_back(current_gnss_pose_data_.pose);
    trajectory_.lidar.push_back(current_lidar_odom_data_.pose);
    trajectory_.length++;
  }
  return true;
}

bool LioBackEndNode::publish_data()
{
  // publish optimized pose
  Eigen::Matrix4f optimized_pose = back_end_->get_odom_to_map() * current_lidar_odom_data_.pose;
  optimized_odom_pub_->publish(optimized_pose.cast<double>(), current_lidar_odom_data_.time);
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
    localization_common::LidarData<pcl::PointXYZ> key_scan;
    back_end_->get_latest_key_scan(key_scan);
    key_scan_pub_->publish(key_scan.point_cloud, key_scan.time);
    localization_common::KeyFrame key_frame;
    back_end_->get_latest_key_frame(key_frame);
    key_frame_pub_->publish(key_frame);
    back_end_->get_latest_key_gnss(key_frame);
    key_gnss_pub_->publish(key_frame);
    // publish optimized key frames
    auto optimized_key_frames = back_end_->get_optimized_key_frames();
    optimized_path_pub_->publish(optimized_key_frames);
    // publish global map
    if (back_end_->has_new_optimized() && global_map_pub_->has_subscribers()) {
      auto cloud = map_generator_->get_global_map(optimized_key_frames, true);
      global_map_pub_->publish(cloud);
    }
  }
  return true;
}

void LioBackEndNode::save_pose(std::ofstream & ofs, const Eigen::Matrix4f & pose)
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

bool LioBackEndNode::save_trajectory(
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

}  // namespace loosely_lio_mapping
