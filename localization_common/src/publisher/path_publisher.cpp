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

#include "localization_common/publisher/path_publisher.hpp"
#include "localization_common/sensor_data_utils.hpp"

namespace localization_common
{
PathPublisher::PathPublisher(
  rclcpp::Node::SharedPtr node, std::string topic_name, std::string frame_id, int buff_size)
: node_(node), frame_id_(frame_id)
{
  publisher_ = node_->create_publisher<nav_msgs::msg::Path>(topic_name, buff_size);
}

void PathPublisher::publish(const std::deque<KeyFrame> & key_frames)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_->get_clock()->now();
  path.header.frame_id = frame_id_;
  for (size_t i = 0; i < key_frames.size(); ++i) {
    auto & key_frame = key_frames.at(i);
    geometry_msgs::msg::PoseStamped pose_stamped;
    rclcpp::Time ros_time(static_cast<uint64_t>(key_frame.time * 1e9));
    pose_stamped.header.stamp = ros_time;
    pose_stamped.header.frame_id = frame_id_;
    pose_stamped.pose.position.x = key_frame.pose(0, 3);
    pose_stamped.pose.position.y = key_frame.pose(1, 3);
    pose_stamped.pose.position.z = key_frame.pose(2, 3);
    Eigen::Quaternionf q = get_quaternion(key_frame.pose);
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();
    path.poses.push_back(pose_stamped);
  }
  publisher_->publish(path);
}

void PathPublisher::publish(const std::vector<LidarFrame> & key_frames)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_->get_clock()->now();
  path.header.frame_id = frame_id_;
  for (size_t i = 0; i < key_frames.size(); ++i) {
    auto & key_frame = key_frames.at(i);
    geometry_msgs::msg::PoseStamped pose_stamped;
    rclcpp::Time ros_time(static_cast<uint64_t>(key_frame.time * 1e9));
    pose_stamped.header.stamp = ros_time;
    pose_stamped.header.frame_id = frame_id_;
    pose_stamped.pose.position.x = key_frame.pose(0, 3);
    pose_stamped.pose.position.y = key_frame.pose(1, 3);
    pose_stamped.pose.position.z = key_frame.pose(2, 3);
    Eigen::Quaterniond q(key_frame.pose.block<3, 3>(0, 0));
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();
    path.poses.push_back(pose_stamped);
  }
  publisher_->publish(path);
}

}  // namespace localization_common
