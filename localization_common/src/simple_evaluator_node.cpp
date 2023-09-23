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

#include "localization_common/simple_evaluator_node.hpp"

#include <filesystem>

namespace localization_common
{
SimpleEvaluatorNode::SimpleEvaluatorNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  // parameters
  std::string data_path;
  node->declare_parameter("data_path", data_path);
  node->declare_parameter("ground_truth_topic", ground_truth_topic_);
  node->declare_parameter("odom_topics", odom_topics_);
  node->declare_parameter("odom_names", odom_names_);
  node->get_parameter("data_path", data_path);
  node->get_parameter("ground_truth_topic", ground_truth_topic_);
  node->get_parameter("odom_topics", odom_topics_);
  node->get_parameter("odom_names", odom_names_);
  // check
  if (data_path == "" || (!std::filesystem::is_directory(data_path))) {
    RCLCPP_FATAL(node->get_logger(), "data_path is invalid: %s", data_path.c_str());
    return;
  }
  if (odom_topics_.size() != odom_names_.size()) {
    RCLCPP_FATAL(node->get_logger(), "the size of odom_topics must equal to odom_names");
    return;
  }
  if (odom_topics_.size() == 0) {
    RCLCPP_FATAL(node->get_logger(), "at least 1 odom topic");
    return;
  }
  trajectory_path_ = data_path + "/trajectory";
  RCLCPP_INFO(node->get_logger(), "path to sotre trajectory: %s", trajectory_path_.c_str());
  // subscriber
  ground_truth_sub_ = std::make_shared<OdometrySubscriber>(node, ground_truth_topic_, 10000);
  for (size_t i = 0; i < odom_names_.size(); i++) {
    auto odom_sub = std::make_shared<OdometrySubscriber>(node, odom_topics_[i], 10000);
    odom_subs_.push_back(odom_sub);
    odom_data_buffers_.emplace_back(1000000);
    RCLCPP_INFO(
      node->get_logger(), "record odom[%s] on topic:%s", odom_names_[i].c_str(),
      odom_topics_[i].c_str());
  }
  // srv
  save_odometry_srv_ = node->create_service<localization_interfaces::srv::SaveOdometry>(
    "save_odometry",
    [this](
      const localization_interfaces::srv::SaveOdometry::Request::SharedPtr /*request*/,
      localization_interfaces::srv::SaveOdometry::Response::SharedPtr response) {
      save_odometry_flag_ = true;
      response->succeed = true;
    });
  //
  run_thread_ = std::make_unique<std::thread>(
    [this]() {
      while (!exit_) {
        run();
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(100ms);
      }
    });
}

SimpleEvaluatorNode::~SimpleEvaluatorNode()
{
  exit_ = true;
  if (run_thread_) {
    run_thread_->join();
  }
}

bool SimpleEvaluatorNode::run()
{
  // ground truth
  std::deque<OdomData> ground_truth_buffer;
  ground_truth_sub_->parse_data(ground_truth_buffer);
  for (auto & data : ground_truth_buffer) {
    ground_truth_buffer_.add_data(data);
    timestamp_buffer_.push_back(data.time);
  }
  // odom data
  for (size_t i = 0; i < odom_topics_.size(); i++) {
    std::deque<OdomData> odom_buffer;
    odom_subs_[i]->parse_data(odom_buffer);
    for (auto & data : odom_buffer) {
      odom_data_buffers_[i].add_data(data);
    }
  }
  if (save_odometry_flag_) {
    save_all_trajectory();
    save_odometry_flag_ = false;
  }
  return true;
}

void SimpleEvaluatorNode::save_pose(std::ofstream & ofs, const Eigen::Matrix4d & pose)
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

bool SimpleEvaluatorNode::save_trajectory(const OdomDataBuffer & buffer, const std::string & name)
{
  std::ofstream trajectory_ofs;
  std::string path = trajectory_path_ + +"/" + name + ".txt";
  trajectory_ofs.open(path, std::ios::app);
  if (!trajectory_ofs) {
    RCLCPP_FATAL(node_->get_logger(), "failed to open path %s", path.c_str());
    return false;
  }
  for (size_t i = 0; i < timestamp_buffer_.size(); ++i) {
    OdomData odom;
    buffer.get_interpolated_data(timestamp_buffer_[i], odom);
    save_pose(trajectory_ofs, odom.pose);
  }
  RCLCPP_INFO(node_->get_logger(), "successed to save odom [%s].", name.c_str());
  return true;
}

bool SimpleEvaluatorNode::save_all_trajectory()
{
  if (std::filesystem::is_directory(trajectory_path_)) {
    std::filesystem::remove_all(trajectory_path_);
  }
  if (!std::filesystem::create_directory(trajectory_path_)) {
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "start to save trajectory");
  // get start time & end time
  double start_time = ground_truth_buffer_.get_start_time();
  double end_time = ground_truth_buffer_.get_end_time();
  for (size_t i = 0; i < odom_topics_.size(); i++) {
    start_time = std::max(start_time, odom_data_buffers_[i].get_start_time());
    end_time = std::min(end_time, odom_data_buffers_[i].get_end_time());
  }
  // remove invalide data
  size_t invalid_cnt = 0;
  // remove ealier data
  while (!timestamp_buffer_.empty() && timestamp_buffer_.front() < start_time) {
    timestamp_buffer_.pop_front();
    invalid_cnt++;
  }
  // remove older data
  while (!timestamp_buffer_.empty() && timestamp_buffer_.back() > end_time) {
    timestamp_buffer_.pop_back();
    invalid_cnt++;
  }
  if (timestamp_buffer_.empty()) {
    RCLCPP_INFO(node_->get_logger(), "failed to save trajectory due to invalid timestamp");
    return false;
  }
  RCLCPP_INFO(
    node_->get_logger(), "remove %ld, total valid timestamp size: %ld", invalid_cnt,
    timestamp_buffer_.size());
  // save ground_truth
  save_trajectory(ground_truth_buffer_, "ground_truth");
  // save odoms
  for (size_t i = 0; i < odom_names_.size(); i++) {
    save_trajectory(odom_data_buffers_[i], odom_names_[i]);
  }
  RCLCPP_INFO(node_->get_logger(), "finish to save all trajectory.");
  return true;
}

}  // namespace localization_common
