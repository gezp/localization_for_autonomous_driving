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

#include "lidar_mapping/loop_closure_node.hpp"

#include <filesystem>

namespace lidar_mapping
{
LoopClosureNode::LoopClosureNode(rclcpp::Node::SharedPtr node)
{
  std::string loop_closure_config;
  std::string data_path;
  node->declare_parameter("loop_closure_config", loop_closure_config);
  node->declare_parameter("data_path", data_path);
  node->get_parameter("loop_closure_config", loop_closure_config);
  node->get_parameter("data_path", data_path);
  RCLCPP_INFO(node->get_logger(), "loop_closure_config: [%s]", loop_closure_config.c_str());
  RCLCPP_INFO(node->get_logger(), "data_path: [%s]", data_path.c_str());
  if (loop_closure_config == "" || (!std::filesystem::exists(loop_closure_config))) {
    RCLCPP_FATAL(node->get_logger(), "loop_closure_config is invalid");
    return;
  }
  if (data_path == "" || (!std::filesystem::is_directory(data_path))) {
    RCLCPP_FATAL(node->get_logger(), "data_path is invalid");
    return;
  }
  // loop closing
  loop_closure_ = std::make_shared<LoopClosure>();
  std::cout << "-----------------Init Loop-Closing Detection-------------------" << std::endl;
  loop_closure_->init_config(loop_closure_config, data_path);
  // subscriber:
  key_scan_sub_ = std::make_shared<localization_common::CloudSubscriber>(node, "key_scan", 1000);
  key_frame_sub_ =
    std::make_shared<localization_common::KeyFrameSubscriber>(node, "key_frame", 1000);
  key_gnss_sub_ = std::make_shared<localization_common::KeyFrameSubscriber>(node, "key_gnss", 1000);
  // publisher
  loop_pose_pub_ =
    std::make_shared<localization_common::LoopPosePublisher>(node, "loop_pose", "map", 100);
  // save map callback
  save_scan_context_srv_ = node->create_service<localization_interfaces::srv::SaveScanContext>(
    "save_scan_context",
    [this](
      const localization_interfaces::srv::SaveScanContext::Request::SharedPtr /*request*/,
      localization_interfaces::srv::SaveScanContext::Response::SharedPtr response) {
      save_scan_context_flag_ = true;
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

LoopClosureNode::~LoopClosureNode()
{
  exit_ = true;
  if (run_thread_) {
    run_thread_->join();
  }
}

bool LoopClosureNode::run()
{
  if (save_scan_context_flag_) {
    // save scan_context
    loop_closure_->save();
    save_scan_context_flag_ = false;
  }
  if (!read_data()) {
    return false;
  }
  while (has_data()) {
    if (!valid_data()) {
      continue;
    }
    loop_closure_->update(current_key_scan_, current_key_frame_, current_key_gnss_);
    publish_data();
  }
  return true;
}

bool LoopClosureNode::read_data()
{
  key_scan_sub_->parse_data(key_scan_buff_);
  key_frame_sub_->parse_data(key_frame_buff_);
  key_gnss_sub_->parse_data(key_gnss_buff_);
  return true;
}

bool LoopClosureNode::has_data()
{
  if (key_scan_buff_.size() == 0 || key_frame_buff_.size() == 0 || key_gnss_buff_.size() == 0) {
    return false;
  }
  return true;
}

bool LoopClosureNode::valid_data()
{
  current_key_scan_ = key_scan_buff_.front();
  current_key_frame_ = key_frame_buff_.front();
  current_key_gnss_ = key_gnss_buff_.front();
  double diff_gnss_time = current_key_frame_.time - current_key_gnss_.time;
  if (diff_gnss_time < -0.05) {
    key_frame_buff_.pop_front();
    return false;
  }
  if (diff_gnss_time > 0.05) {
    key_gnss_buff_.pop_front();
    return false;
  }
  key_scan_buff_.pop_front();
  key_frame_buff_.pop_front();
  key_gnss_buff_.pop_front();
  return true;
}

bool LoopClosureNode::publish_data()
{
  if (loop_closure_->has_new_loop_pose()) {
    loop_pose_pub_->publish(loop_closure_->get_current_loop_pose());
  }
  return true;
}
}  // namespace lidar_mapping
