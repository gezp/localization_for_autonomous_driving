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

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "localization_interfaces/msg/loop_candidate.hpp"
#include "localization_common/sensor_data/loop_candidate.hpp"

namespace localization_common
{
class LoopCandidatePublisher
{
public:
  LoopCandidatePublisher(rclcpp::Node::SharedPtr node, std::string topic_name, int buffer_size);

  void publish(LoopCandidate & loop_candidate);
  bool has_subscribers() {return publisher_->get_subscription_count() > 0;}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<localization_interfaces::msg::LoopCandidate>::SharedPtr publisher_;
};
}  // namespace localization_common
