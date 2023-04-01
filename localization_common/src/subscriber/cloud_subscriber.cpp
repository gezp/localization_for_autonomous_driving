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
  rclcpp::Node::SharedPtr node, std::string topic_name, size_t buff_size)
: node_(node)
{
  subscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_name, buff_size, std::bind(&CloudSubscriber::msg_callback, this, std::placeholders::_1));
}

void CloudSubscriber::msg_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr)
{
  CloudData cloud_data;
  cloud_data.time = rclcpp::Time(cloud_msg_ptr->header.stamp).seconds();
  for (size_t i = 0; i < cloud_msg_ptr->fields.size(); i++) {
    // TODO(all): fix rosbag.
    if (cloud_msg_ptr->fields[i].name == "i") {
      cloud_msg_ptr->fields[i].name = "intensity";
    }
  }
  pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud));
  buff_mutex_.lock();
  new_cloud_data_.push_back(cloud_data);
  buff_mutex_.unlock();
}

void CloudSubscriber::parse_data(std::deque<CloudData> & cloud_data_buff)
{
  buff_mutex_.lock();
  if (new_cloud_data_.size() > 0) {
    cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
    new_cloud_data_.clear();
  }
  buff_mutex_.unlock();
}
}  // namespace localization_common
