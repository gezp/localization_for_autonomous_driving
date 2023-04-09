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

#pragma once

#include <memory>
#include <deque>
//
#include "rclcpp/rclcpp.hpp"
#include "localization_interfaces/srv/save_scan_context.hpp"
#include "localization_common/subscriber/cloud_subscriber.hpp"
#include "localization_common/subscriber/key_frame_subscriber.hpp"
#include "localization_common/publisher/loop_pose_publisher.hpp"
#include "lidar_mapping/loop_closure.hpp"

namespace lidar_mapping
{
class LoopClosureNode
{
public:
  explicit LoopClosureNode(rclcpp::Node::SharedPtr node);
  ~LoopClosureNode();

private:
  bool run();
  bool read_data();
  bool has_data();
  bool valid_data();
  bool publish_data();

private:
  // sub & pub
  std::shared_ptr<localization_common::CloudSubscriber> key_scan_sub_;
  std::shared_ptr<localization_common::KeyFrameSubscriber> key_frame_sub_;
  std::shared_ptr<localization_common::KeyFrameSubscriber> key_gnss_sub_;
  std::shared_ptr<localization_common::LoopPosePublisher> loop_pose_pub_;
  // srv
  rclcpp::Service<localization_interfaces::srv::SaveScanContext>::SharedPtr save_scan_context_srv_;
  bool save_scan_context_flag_{false};
  // loop closing and flow thread
  std::shared_ptr<LoopClosure> loop_closure_;
  std::unique_ptr<std::thread> run_thread_;
  bool exit_{false};
  // data
  std::deque<localization_common::CloudData> key_scan_buff_;
  std::deque<localization_common::KeyFrame> key_frame_buff_;
  std::deque<localization_common::KeyFrame> key_gnss_buff_;

  localization_common::CloudData current_key_scan_;
  localization_common::KeyFrame current_key_frame_;
  localization_common::KeyFrame current_key_gnss_;
};
}  // namespace lidar_mapping
