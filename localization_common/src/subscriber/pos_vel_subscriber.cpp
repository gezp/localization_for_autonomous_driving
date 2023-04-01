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

#include "localization_common/subscriber/pos_vel_subscriber.hpp"

namespace localization_common
{

PosVelSubscriber::PosVelSubscriber(
  rclcpp::Node::SharedPtr node, std::string topic_name, size_t buff_size)
: node_(node)
{
  subscriber_ = node_->create_subscription<localization_interfaces::msg::PosVel>(
    topic_name, buff_size, std::bind(&PosVelSubscriber::msg_callback, this, std::placeholders::_1));
}

void PosVelSubscriber::msg_callback(localization_interfaces::msg::PosVel::SharedPtr pos_vel_ptr)
{
  buff_mutex_.lock();

  PosVelData pos_vel_data;
  pos_vel_data.time = rclcpp::Time(pos_vel_ptr->header.stamp).seconds();

  // a. set the position:
  pos_vel_data.pos.x() = pos_vel_ptr->position.x;
  pos_vel_data.pos.y() = pos_vel_ptr->position.y;
  pos_vel_data.pos.z() = pos_vel_ptr->position.z;

  // b. set the body frame velocity:
  pos_vel_data.vel.x() = pos_vel_ptr->velocity.x;
  pos_vel_data.vel.y() = pos_vel_ptr->velocity.y;
  pos_vel_data.vel.z() = pos_vel_ptr->velocity.z;

  new_pos_vel_data_.push_back(pos_vel_data);

  buff_mutex_.unlock();
}

void PosVelSubscriber::parse_data(std::deque<PosVelData> & pos_vel_data_buff)
{
  buff_mutex_.lock();

  if (new_pos_vel_data_.size() > 0) {
    pos_vel_data_buff.insert(
      pos_vel_data_buff.end(), new_pos_vel_data_.begin(), new_pos_vel_data_.end());
    new_pos_vel_data_.clear();
  }

  buff_mutex_.unlock();
}

}  // namespace localization_common
