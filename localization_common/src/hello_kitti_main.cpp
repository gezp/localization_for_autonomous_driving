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

#include <pcl/common/transforms.h>

#include <cstdlib>

//
#include "localization_common/tf_utils.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
//
#include "localization_common/subscriber/cloud_subscriber.hpp"
#include "localization_common/subscriber/gnss_subscriber.hpp"
#include "localization_common/subscriber/imu_subscriber.hpp"
//
#include "localization_common/publisher/cloud_publisher.hpp"
#include "localization_common/publisher/odometry_publisher.hpp"

using localization_common::CloudData;
using localization_common::GNSSData;
using localization_common::IMUData;

void get_transform_imu_to_map(
  GNSSData & gnss_data, IMUData & imu_data, Eigen::Matrix4f & imu_to_map)
{
  // a. set position:
  imu_to_map(0, 3) = gnss_data.local_E;
  imu_to_map(1, 3) = gnss_data.local_N;
  imu_to_map(2, 3) = gnss_data.local_U;
  // b. set orientation:
  imu_to_map.block<3, 3>(0, 0) = imu_data.orientation.matrix().cast<float>();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("hello_kitti_node");

  // get TF:
  auto imu_frame_id = "imu_link";
  auto lidar_frame_id = "velo_link";
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  auto lidar_to_map_tf_pub_ptr = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  // subscribe to topics:
  auto cloud_sub_ptr =
    std::make_shared<localization_common::CloudSubscriber>(node, "/kitti/velo/pointcloud", 100000);
  auto imu_sub_ptr =
    std::make_shared<localization_common::IMUSubscriber>(node, "/kitti/oxts/imu", 1000000);
  auto gnss_sub_ptr =
    std::make_shared<localization_common::GNSSSubscriber>(node, "/kitti/oxts/gps/fix", 1000000);
  // register publishers:
  auto cloud_pub_ptr =
    std::make_shared<localization_common::CloudPublisher>(node, "current_scan", "map", 100);
  auto odom_pub_ptr = std::make_shared<localization_common::OdometryPublisher>(
    node, "lidar_odom", "map", "velo_link", 100);

  std::deque<CloudData> cloud_data_buff;
  std::deque<IMUData> imu_data_buff;
  std::deque<GNSSData> gnss_data_buff;

  Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f imu_to_map = Eigen::Matrix4f::Identity();

  bool transform_received = false;

  rclcpp::Rate loop_rate(100);
  while (rclcpp::ok()) {
    cloud_sub_ptr->parse_data(cloud_data_buff);
    imu_sub_ptr->parse_data(imu_data_buff);
    gnss_sub_ptr->parse_data(gnss_data_buff);

    if (!transform_received) {
      if (localization_common::lookup_in_tf_buffer(
          tf_buffer, imu_frame_id, lidar_frame_id, lidar_to_imu))
      {
        transform_received = true;
      }
    } else {
      while (!cloud_data_buff.empty() && !imu_data_buff.empty() && !gnss_data_buff.empty()) {
        CloudData cloud_data = cloud_data_buff.front();
        IMUData imu_data = imu_data_buff.front();
        GNSSData gnss_data = gnss_data_buff.front();

        double d_time = cloud_data.time - imu_data.time;

        if (d_time < -0.05) {
          cloud_data_buff.pop_front();
        } else if (d_time > 0.05) {
          imu_data_buff.pop_front();
          gnss_data_buff.pop_front();
        } else {
          cloud_data_buff.pop_front();
          imu_data_buff.pop_front();
          gnss_data_buff.pop_front();

          get_transform_imu_to_map(gnss_data, imu_data, imu_to_map);

          // lidar pose in map frame:
          Eigen::Matrix4f lidar_odometry = imu_to_map * lidar_to_imu;
          // lidar measurement in map frame:
          pcl::transformPointCloud(*cloud_data.cloud, *cloud_data.cloud, lidar_odometry);

          cloud_pub_ptr->publish(cloud_data.cloud);
          odom_pub_ptr->publish(lidar_odometry);

          // publish TF: lidar -> map
          auto msg = localization_common::to_transform_stamped_msg(lidar_odometry, cloud_data.time);
          msg.header.frame_id = "map";
          msg.child_frame_id = "velo_link";
          lidar_to_map_tf_pub_ptr->sendTransform(msg);
        }
      }
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
