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

#include "localization_common/publisher/loop_pose_publisher.hpp"
#include "localization_common/sensor_data_utils.hpp"

namespace localization_common
{
LoopPosePublisher::LoopPosePublisher(
  rclcpp::Node::SharedPtr node, std::string topic_name, std::string frame_id, int buff_size)
: node_(node), frame_id_(frame_id)
{
  publisher_ =
    node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(topic_name, buff_size);
}

void LoopPosePublisher::publish(LoopPose & loop_pose)
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose_stamped;

  rclcpp::Time ros_time(static_cast<uint64_t>(loop_pose.time * 1e9));
  pose_stamped.header.stamp = ros_time;
  pose_stamped.header.frame_id = frame_id_;

  pose_stamped.pose.pose.position.x = loop_pose.pose(0, 3);
  pose_stamped.pose.pose.position.y = loop_pose.pose(1, 3);
  pose_stamped.pose.pose.position.z = loop_pose.pose(2, 3);

  Eigen::Quaternionf q = get_quaternion(loop_pose.pose);
  pose_stamped.pose.pose.orientation.x = q.x();
  pose_stamped.pose.pose.orientation.y = q.y();
  pose_stamped.pose.pose.orientation.z = q.z();
  pose_stamped.pose.pose.orientation.w = q.w();

  pose_stamped.pose.covariance[0] = static_cast<double>(loop_pose.index0);
  pose_stamped.pose.covariance[1] = static_cast<double>(loop_pose.index1);

  publisher_->publish(pose_stamped);
}

}  // namespace localization_common
