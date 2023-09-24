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
  std::string reference_odom_name;
  node->declare_parameter("trajectory_path", trajectory_path_);
  node->declare_parameter("odom_names", odom_names_);
  node->declare_parameter("odom_topics", odom_topics_);
  node->declare_parameter("reference_odom_name", reference_odom_name);
  node->get_parameter("trajectory_path", trajectory_path_);
  node->get_parameter("odom_names", odom_names_);
  node->get_parameter("odom_topics", odom_topics_);
  node->get_parameter("reference_odom_name", reference_odom_name);
  // trajectory_path
  if (trajectory_path_ == "") {
    RCLCPP_FATAL(node->get_logger(), "trajectory_path is empty.");
    return;
  }
  if (!std::filesystem::is_directory(std::filesystem::path(trajectory_path_).parent_path())) {
    RCLCPP_FATAL(node->get_logger(), "trajectory_path is invalid: %s", trajectory_path_.c_str());
    return;
  }
  RCLCPP_INFO(node->get_logger(), "trajectory storage path: %s", trajectory_path_.c_str());
  // odom_names & odom_topics
  if (odom_topics_.size() != odom_names_.size()) {
    RCLCPP_FATAL(node->get_logger(), "the size of odom_topics must equal to odom_names");
    return;
  }
  if (odom_names_.size() == 0) {
    RCLCPP_FATAL(node->get_logger(), "at least 1 odom topic");
    return;
  }
  if (!check_unique_element(odom_names_) || !check_unique_element(odom_topics_)) {
    RCLCPP_FATAL(node->get_logger(), "the elements in odom_names and odom_topics must be unique");
    return;
  }
  // create subscriber
  for (size_t i = 0; i < odom_names_.size(); i++) {
    auto odom_sub = std::make_shared<OdometrySubscriber>(node, odom_topics_[i], 10000);
    odom_subs_.push_back(odom_sub);
    odom_data_buffers_.emplace_back(1000000);
    RCLCPP_INFO(
      node->get_logger(), "record odom[%s] on topic:%s", odom_names_[i].c_str(),
      odom_topics_[i].c_str());
  }
  // reference_odom
  for (size_t i = 0; i < odom_names_.size(); i++) {
    if (odom_names_[i] == reference_odom_name) {
      reference_odom_index_ = i;
    }
  }
  RCLCPP_INFO(
    node->get_logger(), "use odom[%s] as timestamp reference.",
    odom_names_[reference_odom_index_].c_str());
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
  // update odom data
  for (size_t i = 0; i < odom_topics_.size(); i++) {
    std::deque<OdomData> odom_buffer;
    odom_subs_[i]->parse_data(odom_buffer);
    for (auto & data : odom_buffer) {
      odom_data_buffers_[i].add_data(data);
      // timestamp buffer
      if (i == 0) {
        timestamp_buffer_.push_back(data.time);
      }
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

bool SimpleEvaluatorNode::save_trajectory(OdomDataBuffer & buffer, const std::string & name)
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
    RCLCPP_INFO(node_->get_logger(), "failed to create trajectory directory.");
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "start to save trajectory");
  // get start time & end time
  double start_time = 0;
  double end_time = std::numeric_limits<double>::max();
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
  // save odoms
  for (size_t i = 0; i < odom_names_.size(); i++) {
    save_trajectory(odom_data_buffers_[i], odom_names_[i]);
  }
  RCLCPP_INFO(node_->get_logger(), "finish to save all trajectory.");
  return true;
}

bool SimpleEvaluatorNode::check_unique_element(const std::vector<std::string> & v)
{
  std::set<std::string> s(v.begin(), v.end());
  return s.size() == v.size();
}

}  // namespace localization_common
