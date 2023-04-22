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
  node->declare_parameter("max_miss_time", max_miss_time_);
  node->declare_parameter("show_miss_data_info", show_miss_data_info_);
  node->get_parameter("data_path", data_path);
  node->get_parameter("ground_truth_topic", ground_truth_topic_);
  node->get_parameter("odom_topics", odom_topics_);
  node->get_parameter("odom_names", odom_names_);
  node->get_parameter("max_miss_time", max_miss_time_);
  node->get_parameter("show_miss_data_info", show_miss_data_info_);
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
    odom_data_buffs_.push_back({});
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
  ground_truth_sub_->parse_data(ground_truth_data_buff_);
  for (size_t i = 0; i < odom_topics_.size(); i++) {
    odom_subs_[i]->parse_data(odom_data_buffs_[i]);
  }
  if (save_odometry_flag_) {
    save_trajectory();
    save_odometry_flag_ = false;
  }
  return true;
}

void SimpleEvaluatorNode::save_pose(std::ofstream & ofs, const Eigen::Matrix4f & pose)
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

bool SimpleEvaluatorNode::save_trajectory()
{
  if (std::filesystem::is_directory(trajectory_path_)) {
    std::filesystem::remove_all(trajectory_path_);
  }
  if (!std::filesystem::create_directory(trajectory_path_)) {
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "start to save trajectory");
  // remove ground truth with earlier time than odom.
  double start_time = 0;
  for (size_t i = 0; i < odom_topics_.size(); i++) {
    start_time = std::max(start_time, odom_data_buffs_[i].front().time);
  }
  size_t earlier_cnt = 0;
  while (!ground_truth_data_buff_.empty() && (ground_truth_data_buff_.front().time < start_time)) {
    ground_truth_data_buff_.pop_front();
    earlier_cnt++;
  }
  // save ground_truth
  std::ofstream ground_truth_ofs;
  std::string path = trajectory_path_ + "/ground_truth.txt";
  ground_truth_ofs.open(path, std::ios::app);
  if (!ground_truth_ofs) {
    RCLCPP_FATAL(node_->get_logger(), "failed to open path %s", path.c_str());
    return false;
  }
  for (size_t i = 0; i < ground_truth_data_buff_.size(); ++i) {
    save_pose(ground_truth_ofs, ground_truth_data_buff_[i].pose);
  }
  RCLCPP_INFO(
    node_->get_logger(), "save ground truth, remove %ld earlier data, total valid size: %ld",
    earlier_cnt, ground_truth_data_buff_.size());
  // save odoms
  for (size_t i = 0; i < odom_names_.size(); i++) {
    std::ofstream lidar_odom_ofs;
    path = trajectory_path_ + "/" + odom_names_[i] + ".txt";
    lidar_odom_ofs.open(path, std::ios::app);
    if (!lidar_odom_ofs) {
      RCLCPP_FATAL(node_->get_logger(), "failed to open path %s", path.c_str());
      return false;
    }
    int nearest_idx = 0;
    int miss_cnt = 0;
    auto & odom_data = odom_data_buffs_[i];
    auto odom_name = odom_names_[i].c_str();
    for (size_t j = 0; j < ground_truth_data_buff_.size(); ++j) {
      while (odom_data.size() - nearest_idx > 1) {
        auto dt0 = fabs(odom_data[nearest_idx].time - ground_truth_data_buff_[j].time);
        auto dt1 = fabs(odom_data[nearest_idx + 1].time - ground_truth_data_buff_[j].time);
        if (dt0 < dt1) {
          break;
        }
        nearest_idx++;
      }
      auto dt = odom_data[nearest_idx].time - ground_truth_data_buff_[j].time;
      // miss
      if (dt > max_miss_time_) {
        if (show_miss_data_info_) {
          RCLCPP_WARN(node_->get_logger(), "odom[%s] miss index %d, dt: %lf.", odom_name, j, dt);
        }
        miss_cnt++;
      }
      save_pose(lidar_odom_ofs, odom_data[nearest_idx].pose);
    }
    RCLCPP_INFO(node_->get_logger(), "save odom[%s], total miss: %d.", odom_name, miss_cnt);
  }
  RCLCPP_INFO(node_->get_logger(), "finish to save all trajectory.");
  return true;
}

}  // namespace localization_common
