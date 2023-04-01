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

#include "localization_common/tf_utils.hpp"

namespace localization_common
{

geometry_msgs::msg::TransformStamped to_transform_stamped_msg(
  Eigen::Vector3d t, Eigen::Quaterniond q, double time)
{
  geometry_msgs::msg::TransformStamped msg;
  rclcpp::Time ros_time(static_cast<uint64_t>(time * 1e9));
  msg.header.stamp = ros_time;
  msg.transform.translation.x = t.x();
  msg.transform.translation.y = t.y();
  msg.transform.translation.z = t.z();
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
  msg.transform.rotation.w = q.w();
  return msg;
}

geometry_msgs::msg::TransformStamped to_transform_stamped_msg(Eigen::Matrix4f pose, double time)
{
  geometry_msgs::msg::TransformStamped msg;
  rclcpp::Time ros_time(static_cast<uint64_t>(time * 1e9));
  Eigen::Quaternionf q(pose.block<3, 3>(0, 0));
  msg.header.stamp = ros_time;
  msg.transform.translation.x = pose(0, 3);
  msg.transform.translation.y = pose(1, 3);
  msg.transform.translation.z = pose(2, 3);
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
  msg.transform.rotation.w = q.w();
  return msg;
}

bool lookup_in_tf_buffer(
  std::shared_ptr<tf2_ros::Buffer> buffer, std::string base_frame_id, std::string child_frame_id,
  Eigen::Matrix4f & transform_matrix)
{
  try {
    auto transform = buffer->lookupTransform(base_frame_id, child_frame_id, tf2::TimePointZero);
    transform_matrix = tf2::transformToEigen(transform).matrix().cast<float>();
    return true;
  } catch (const tf2::TransformException & ex) {
    return false;
  }
}

}  // namespace localization_common
