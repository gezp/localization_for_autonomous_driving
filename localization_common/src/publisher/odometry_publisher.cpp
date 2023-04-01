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

#include "localization_common/publisher/odometry_publisher.hpp"

namespace localization_common
{
OdometryPublisher::OdometryPublisher(
  rclcpp::Node::SharedPtr node, std::string topic_name, std::string base_frame_id,
  std::string child_frame_id, int buff_size)
: node_(node)
{
  publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(topic_name, buff_size);
  odometry_.header.frame_id = base_frame_id;
  odometry_.child_frame_id = child_frame_id;
}

void OdometryPublisher::publish(
  const Eigen::Matrix4f & transform_matrix, const VelocityData & velocity_data, rclcpp::Time time)
{
  odometry_.header.stamp = time;

  // set the position
  odometry_.pose.pose.position.x = transform_matrix(0, 3);
  odometry_.pose.pose.position.y = transform_matrix(1, 3);
  odometry_.pose.pose.position.z = transform_matrix(2, 3);

  Eigen::Quaternionf q;
  q = transform_matrix.block<3, 3>(0, 0);
  q.normalize();
  odometry_.pose.pose.orientation.x = q.x();
  odometry_.pose.pose.orientation.y = q.y();
  odometry_.pose.pose.orientation.z = q.z();
  odometry_.pose.pose.orientation.w = q.w();

  // set the twist:
  odometry_.twist.twist.linear.x = velocity_data.linear_velocity.x();
  odometry_.twist.twist.linear.y = velocity_data.linear_velocity.y();
  odometry_.twist.twist.linear.z = velocity_data.linear_velocity.z();
  odometry_.twist.twist.angular.x = velocity_data.angular_velocity.x();
  odometry_.twist.twist.angular.y = velocity_data.angular_velocity.y();
  odometry_.twist.twist.angular.z = velocity_data.angular_velocity.z();

  publisher_->publish(odometry_);
}

void OdometryPublisher::publish(const Eigen::Matrix4f & transform_matrix, double time)
{
  rclcpp::Time ros_time(static_cast<uint64_t>(time * 1e9));
  publish(transform_matrix, velocity_data_, ros_time);
}

void OdometryPublisher::publish(const Eigen::Matrix4f & transform_matrix)
{
  publish(transform_matrix, velocity_data_, node_->get_clock()->now());
}

void OdometryPublisher::publish(
  const Eigen::Matrix4f & transform_matrix, const VelocityData & velocity_data, double time)
{
  rclcpp::Time ros_time(static_cast<uint64_t>(time * 1e9));
  publish(transform_matrix, velocity_data, ros_time);
}

void OdometryPublisher::publish(
  const Eigen::Matrix4f & transform_matrix, const VelocityData & velocity_data)
{
  publish(transform_matrix, velocity_data, node_->get_clock()->now());
}

void OdometryPublisher::publish(
  const Eigen::Matrix4f & transform_matrix, const Eigen::Vector3f & vel, double time)
{
  rclcpp::Time ros_time(static_cast<uint64_t>(time * 1e9));
  VelocityData velocity_data;
  velocity_data.linear_velocity = vel;
  publish(transform_matrix, velocity_data, ros_time);
}

void OdometryPublisher::publish(
  const Eigen::Matrix4f & transform_matrix, const Eigen::Vector3f & vel)
{
  VelocityData velocity_data;
  velocity_data.linear_velocity = vel;
  publish(transform_matrix, velocity_data, node_->get_clock()->now());
}

}  // namespace localization_common
