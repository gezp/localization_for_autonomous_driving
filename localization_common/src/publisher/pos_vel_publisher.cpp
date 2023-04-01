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

#include "localization_common/publisher/pos_vel_publisher.hpp"

namespace localization_common
{

PosVelPublisher::PosVelPublisher(
  rclcpp::Node::SharedPtr node, std::string topic_name, std::string base_frame_id,
  std::string child_frame_id, int buff_size)
: node_(node)
{
  publisher_ = node_->create_publisher<localization_interfaces::msg::PosVel>(topic_name, buff_size);
  pos_vel_msg_.header.frame_id = base_frame_id;
  pos_vel_msg_.child_frame_id = child_frame_id;
}

void PosVelPublisher::publish(const PosVelData & pos_vel_data, const double & time)
{
  rclcpp::Time ros_time(static_cast<uint64_t>(time * 1e9));
  publish_data(pos_vel_data, ros_time);
}

void PosVelPublisher::publish(const PosVelData & pos_vel_data)
{
  publish_data(pos_vel_data, node_->get_clock()->now());
}

bool PosVelPublisher::has_subscribers() {return publisher_->get_subscription_count() > 0;}

void PosVelPublisher::publish_data(const PosVelData & pos_vel_data, rclcpp::Time time)
{
  pos_vel_msg_.header.stamp = time;

  // a. set position
  pos_vel_msg_.position.x = pos_vel_data.pos.x();
  pos_vel_msg_.position.y = pos_vel_data.pos.y();
  pos_vel_msg_.position.z = pos_vel_data.pos.z();

  // b. set velocity:
  pos_vel_msg_.velocity.x = pos_vel_data.vel.x();
  pos_vel_msg_.velocity.y = pos_vel_data.vel.y();
  pos_vel_msg_.velocity.z = pos_vel_data.vel.z();

  publisher_->publish(pos_vel_msg_);
}

}  // namespace localization_common
