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

#include "localization_common/kitti_preprocess_node.hpp"

#include "localization_common/data_synchronization.hpp"
#include "localization_common/sensor_data_utils.hpp"
#include "localization_common/sensor_data/lidar_data.hpp"

namespace localization_common
{
KittiPreprocessNode::KittiPreprocessNode(rclcpp::Node::SharedPtr node)
{
  node->declare_parameter("use_manual_gnss_datum", use_manual_gnss_datum_);
  node->declare_parameter("gnss_datum", gnss_datum_);
  node->get_parameter("use_manual_gnss_datum", use_manual_gnss_datum_);
  node->get_parameter("gnss_datum", gnss_datum_);
  if (gnss_datum_.size() != 3) {
    RCLCPP_FATAL(
      node->get_logger(), "gnss_datum's size must be 3 for (latitude, longitude, altitude)");
    return;
  }
  imu_frame_id_ = "imu_link";
  lidar_frame_id_ = "velo_link";
  // subscriber
  cloud_sub_ = std::make_shared<CloudSubscriber<pcl::PointXYZ>>(
    node, "/kitti/velo/pointcloud",
    10000);
  imu_sub_ = std::make_shared<ImuSubscriber>(node, "/kitti/oxts/imu", 10000);
  velocity_sub_ = std::make_shared<VelocitySubscriber>(node, "/kitti/oxts/gps/vel", 10000);
  gnss_sub_ = std::make_shared<GnssSubscriber>(node, "/kitti/oxts/gps/fix", 10000);
  if (use_manual_gnss_datum_) {
    gnss_sub_->set_gnss_datum(gnss_datum_[0], gnss_datum_[1], gnss_datum_[2]);
  }
  // publisher
  cloud_pub_ = std::make_shared<CloudPublisher<pcl::PointXYZ>>(
    node, "synced_cloud",
    base_link_frame_id_, 100);
  gnss_pose_pub_ =
    std::make_shared<OdometryPublisher>(node, "synced_gnss/pose", "map", base_link_frame_id_, 100);
  imu_pub_ = std::make_shared<ImuPublisher>(node, "synced_imu", imu_frame_id_, 100);
  pos_vel_pub_ =
    std::make_shared<PosVelPublisher>(node, "synced_pos_vel", "map", imu_frame_id_, 100);
  // tf
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // motion compensation for lidar measurement:
  distortion_adjust_ = std::make_shared<DistortionAdjust>();
  //
  run_thread_ = std::make_unique<std::thread>(
    [this]() {
      while (run_flag_) {
        if (!run()) {
          using namespace std::chrono_literals;
          std::this_thread::sleep_for(10ms);
        }
      }
    });
}

bool KittiPreprocessNode::run()
{
  if (!init_calibration()) {
    return false;
  }
  if (!read_data()) {
    return false;
  }
  while (has_data()) {
    if (!valid_data()) {
      continue;
    }
    transform_data();
    publish_data();
  }
  return true;
}

bool KittiPreprocessNode::read_data()
{
  static std::deque<ImuData2> unsynced_imu_;
  static std::deque<VelocityData> unsynced_velocity_;
  static std::deque<GnssData> unsynced_gnss_;

  // fetch lidar measurements from buffer:
  cloud_sub_->parse_data(lidar_data_buff_);
  imu_sub_->parse_data(unsynced_imu_);
  velocity_sub_->parse_data(unsynced_velocity_);
  gnss_sub_->parse_data(unsynced_gnss_);

  static bool sensor_inited = false;
  if (!sensor_inited && lidar_data_buff_.size() < 5) {
    return false;
  }

  if (lidar_data_buff_.size() == 0) {
    return false;
  }

  // use timestamp of lidar measurement as reference:
  double cloud_time = lidar_data_buff_.front().time;
  // sync IMU, velocity and GNSS with lidar measurement:
  // find the two closest measurement around lidar measurement time
  // then use linear interpolation to generate synced measurement:
  bool valid_imu = sync_imu_data2(unsynced_imu_, imu_data_buff_, cloud_time);
  bool valid_velocity = sync_velocity_data(unsynced_velocity_, velocity_data_buff_, cloud_time);
  bool valid_gnss = sync_gnss_data(unsynced_gnss_, gnss_data_buff_, cloud_time);

  // only mark lidar as 'inited' when all the three sensors are synced:
  if (!sensor_inited) {
    if (!valid_imu || !valid_velocity || !valid_gnss) {
      lidar_data_buff_.pop_front();
      return false;
    }
    sensor_inited = true;
  }

  return true;
}

bool KittiPreprocessNode::init_calibration()
{
  // lookup imu pose in lidar frame:
  static bool calibration_received = false;
  if (calibration_received) {
    return true;
  }
  if (!localization_common::lookup_in_tf_buffer(
      tf_buffer_, imu_frame_id_, base_link_frame_id_, base_link_to_imu_))
  {
    return false;
  }
  if (!localization_common::lookup_in_tf_buffer(
      tf_buffer_, lidar_frame_id_, base_link_frame_id_, base_link_to_lidar_))
  {
    return false;
  }
  if (!localization_common::lookup_in_tf_buffer(
      tf_buffer_, imu_frame_id_, lidar_frame_id_, lidar_to_imu_))
  {
    return false;
  }
  calibration_received = true;
  return true;
}

bool KittiPreprocessNode::has_data()
{
  if (lidar_data_buff_.size() == 0) {
    return false;
  }
  if (imu_data_buff_.size() == 0) {
    return false;
  }
  if (velocity_data_buff_.size() == 0) {
    return false;
  }
  if (gnss_data_buff_.size() == 0) {
    return false;
  }
  return true;
}

bool KittiPreprocessNode::valid_data()
{
  current_lidar_data_ = lidar_data_buff_.front();
  current_imu_data_ = imu_data_buff_.front();
  current_velocity_data_ = velocity_data_buff_.front();
  current_gnss_data_ = gnss_data_buff_.front();

  double diff_imu_time = current_lidar_data_.time - current_imu_data_.time;
  double diff_velocity_time = current_lidar_data_.time - current_velocity_data_.time;
  double diff_gnss_time = current_lidar_data_.time - current_gnss_data_.time;
  if (diff_imu_time < -0.05 || diff_velocity_time < -0.05 || diff_gnss_time < -0.05) {
    lidar_data_buff_.pop_front();
    return false;
  }
  if (diff_imu_time > 0.05) {
    imu_data_buff_.pop_front();
    return false;
  }
  if (diff_velocity_time > 0.05) {
    velocity_data_buff_.pop_front();
    return false;
  }
  if (diff_gnss_time > 0.05) {
    gnss_data_buff_.pop_front();
    return false;
  }
  lidar_data_buff_.pop_front();
  imu_data_buff_.pop_front();
  velocity_data_buff_.pop_front();
  gnss_data_buff_.pop_front();
  return true;
}

bool KittiPreprocessNode::transform_data()
{
  // motion compensation for lidar measurements:
  auto lidar_velocity = transform_velocity_data(current_velocity_data_, lidar_to_imu_);
  distortion_adjust_->set_motion_info(0.1, lidar_velocity);
  distortion_adjust_->adjust_cloud(
    current_lidar_data_.point_cloud,
    current_lidar_data_.point_cloud);
  pcl::transformPointCloud(
    *current_lidar_data_.point_cloud, *current_lidar_data_.point_cloud,
    base_link_to_lidar_.inverse());
  // get reference pose (position from GNSS, orientation from IMU)
  gnss_pose_ = Eigen::Matrix4f::Identity();
  gnss_pose_(0, 3) = current_gnss_data_.local_E;
  gnss_pose_(1, 3) = current_gnss_data_.local_N;
  gnss_pose_(2, 3) = current_gnss_data_.local_U;
  gnss_pose_.block<3, 3>(0, 0) = current_imu_data_.orientation.matrix().cast<float>();
  gnss_pose_ = gnss_pose_ * base_link_to_imu_;
  // set synced pos vel (in imu frame)
  pos_vel_.pos.x() = current_gnss_data_.local_E;
  pos_vel_.pos.y() = current_gnss_data_.local_N;
  pos_vel_.pos.z() = current_gnss_data_.local_U;
  pos_vel_.vel = current_velocity_data_.linear_velocity;
  return true;
}

bool KittiPreprocessNode::publish_data()
{
  auto velocity = transform_velocity_data(current_velocity_data_, base_link_to_imu_);
  cloud_pub_->publish(current_lidar_data_.point_cloud, current_lidar_data_.time);
  gnss_pose_pub_->publish(gnss_pose_, velocity, current_lidar_data_.time);
  ImuData imu_data;
  imu_data.time = current_imu_data_.time;
  imu_data.linear_acceleration = current_imu_data_.linear_acceleration;
  imu_data.angular_velocity = current_imu_data_.angular_velocity;
  imu_pub_->publish(imu_data, current_lidar_data_.time);
  pos_vel_pub_->publish(pos_vel_, current_lidar_data_.time);
  return true;
}
}  // namespace localization_common
