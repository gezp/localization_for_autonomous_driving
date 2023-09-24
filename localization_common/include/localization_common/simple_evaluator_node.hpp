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

#include <thread>
#include <memory>
#include <deque>
#include <string>
#include <vector>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
//
#include "localization_interfaces/srv/save_odometry.hpp"
#include "localization_common/subscriber/odometry_subscriber.hpp"
#include "localization_common/odom_data_buffer.hpp"

namespace localization_common
{
class SimpleEvaluatorNode
{
public:
  explicit SimpleEvaluatorNode(rclcpp::Node::SharedPtr node);
  ~SimpleEvaluatorNode();

private:
  bool run();
  void save_pose(std::ofstream & ofs, const Eigen::Matrix4d & pose);
  bool save_trajectory();
  bool check_unique_element(const std::vector<std::string> & v);

private:
  rclcpp::Node::SharedPtr node_;
  // odom subscribers
  std::vector<std::string> odom_topics_;
  std::vector<std::string> odom_names_;
  std::vector<std::shared_ptr<OdometrySubscriber>> odom_subs_;
  // srv
  rclcpp::Service<localization_interfaces::srv::SaveOdometry>::SharedPtr save_odometry_srv_;
  bool save_odometry_flag_{false};
  // data
  std::vector<OdomDataBuffer> odom_data_buffers_;
  size_t reference_odom_index_{0};
  //
  std::string trajectory_path_;
  std::unique_ptr<std::thread> run_thread_;
  bool exit_{false};
};
}  // namespace localization_common
