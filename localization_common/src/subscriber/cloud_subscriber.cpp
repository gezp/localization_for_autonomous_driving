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

#include "localization_common/subscriber/cloud_subscriber.hpp"

namespace localization_common
{

CloudSubscriber::CloudSubscriber(
  rclcpp::Node::SharedPtr node, std::string topic_name, size_t buffer_size)
: node_(node)
{
  auto msg_callback = [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr) {
      sensor_msgs::msg::PointCloud2 msg = *msg_ptr;
      // TODO(gezp): fix rosbag.
      for (size_t i = 0; i < msg.fields.size(); i++) {
        if (msg.fields[i].name == "i") {
          msg.fields[i].name = "intensity";
        }
      }
      buffer_mutex_.lock();
      buffer_.push_back(msg);
      buffer_mutex_.unlock();
    };
  subscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_name, buffer_size, msg_callback);
}

}  // namespace localization_common