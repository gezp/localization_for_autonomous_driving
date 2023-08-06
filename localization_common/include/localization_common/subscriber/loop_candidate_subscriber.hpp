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

#include <deque>
#include <mutex>
#include <thread>
#include <string>

#include "localization_interfaces/msg/loop_candidate.hpp"
#include "localization_common/sensor_data/loop_candidate.hpp"
#include "rclcpp/rclcpp.hpp"

namespace localization_common
{
class LoopCandidateSubscriber
{
public:
  LoopCandidateSubscriber(rclcpp::Node::SharedPtr node, std::string topic_name, size_t buff_size);
  void parse_data(std::deque<LoopCandidate> & loop_candidate_buff);

private:
  void msg_callback(
    const localization_interfaces::msg::LoopCandidate::SharedPtr loop_candidate_msg_ptr);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<localization_interfaces::msg::LoopCandidate>::SharedPtr subscriber_;
  std::deque<LoopCandidate> new_loop_candidate_;

  std::mutex buff_mutex_;
};
}  // namespace localization_common
