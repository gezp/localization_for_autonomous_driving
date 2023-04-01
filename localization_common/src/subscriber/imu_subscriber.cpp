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

/*
 * @Description: 订阅imu数据
 * @Author: Ren Qian
 * @Date: 2019-06-14 16:44:18
 */
#include "localization_common/subscriber/imu_subscriber.hpp"

namespace localization_common
{
IMUSubscriber::IMUSubscriber(rclcpp::Node::SharedPtr node, std::string topic_name, size_t buff_size)
: node_(node)
{
  subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    topic_name, buff_size, std::bind(&IMUSubscriber::msg_callback, this, std::placeholders::_1));
}

void IMUSubscriber::msg_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg_ptr)
{
  IMUData imu_data;
  imu_data.time = rclcpp::Time(imu_msg_ptr->header.stamp).seconds();

  imu_data.linear_acceleration[0] = imu_msg_ptr->linear_acceleration.x;
  imu_data.linear_acceleration[1] = imu_msg_ptr->linear_acceleration.y;
  imu_data.linear_acceleration[2] = imu_msg_ptr->linear_acceleration.z;

  imu_data.angular_velocity[0] = imu_msg_ptr->angular_velocity.x;
  imu_data.angular_velocity[1] = imu_msg_ptr->angular_velocity.y;
  imu_data.angular_velocity[2] = imu_msg_ptr->angular_velocity.z;

  auto & q = imu_msg_ptr->orientation;
  imu_data.orientation = Eigen::Quaterniond(q.w, q.x, q.y, q.z);

  buff_mutex_.lock();
  new_imu_data_.push_back(imu_data);
  buff_mutex_.unlock();
}

void IMUSubscriber::parse_data(std::deque<IMUData> & imu_data_buff)
{
  buff_mutex_.lock();
  if (new_imu_data_.size() > 0) {
    imu_data_buff.insert(imu_data_buff.end(), new_imu_data_.begin(), new_imu_data_.end());
    new_imu_data_.clear();
  }
  buff_mutex_.unlock();
}
}  // namespace localization_common
