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

namespace localization_common
{
DataPretreatNode::DataPretreatNode(rclcpp::Node::SharedPtr node)
{
  // subscribe to
  // a. raw Velodyne measurement
  // b. raw GNSS/IMU measurement
  // publish
  // a. undistorted Velodyne measurement
  // b. lidar pose in map frame
  std::string cloud_topic = "synced_cloud";
  node->declare_parameter("cloud_topic", cloud_topic);
  node->get_parameter("cloud_topic", cloud_topic);
  // subscriber
  // a. velodyne measurement:
  cloud_sub_ = std::make_shared<CloudSubscriber>(node, "/kitti/velo/pointcloud", 100000);
  // b. OXTS IMU:
  imu_sub_ = std::make_shared<IMUSubscriber>(node, "/kitti/oxts/imu", 1000000);
  // c. OXTS velocity:
  velocity_sub_ = std::make_shared<VelocitySubscriber>(node, "/kitti/oxts/gps/vel", 1000000);
  // d. OXTS GNSS:
  gnss_sub_ = std::make_shared<GNSSSubscriber>(node, "/kitti/oxts/gps/fix", 1000000);
  imu_frame_id_ = "imu_link";
  lidar_frame_id_ = "velo_link";
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // publisher
  cloud_pub_ = std::make_shared<CloudPublisher>(node, cloud_topic, "velo_link", 100);
  gnss_pub_ = std::make_shared<OdometryPublisher>(node, "synced_gnss", "map", "velo_link", 100);
  imu_pub_ = std::make_shared<IMUPublisher>(node, "synced_imu", "imu_link", 100);
  pos_vel_pub_ = std::make_shared<PosVelPublisher>(node, "synced_pos_vel", "map", "imu_link", 100);
  // motion compensation for lidar measurement:
  distortion_adjust_ = std::make_shared<DistortionAdjust>();
  //
  run_thread_ = std::make_unique<std::thread>(
    [this]() {
      while (run_flag_) {
        if (!run()) {
          using namespace std::chrono_literals;
          std::this_thread::sleep_for(100ms);
        }
      }
    });
}

bool DataPretreatNode::run()
{
  if (!read_data()) {
    return false;
  }
  if (!init_calibration()) {
    return false;
  }
  if (!init_gnss()) {
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

bool DataPretreatNode::read_data()
{
  static std::deque<IMUData> unsynced_imu_;
  static std::deque<VelocityData> unsynced_velocity_;
  static std::deque<GNSSData> unsynced_gnss_;

  // fetch lidar measurements from buffer:
  cloud_sub_->parse_data(cloud_data_buff_);
  imu_sub_->parse_data(unsynced_imu_);
  velocity_sub_->parse_data(unsynced_velocity_);
  gnss_sub_->parse_data(unsynced_gnss_);

  if (cloud_data_buff_.size() == 0) {
    return false;
  }

  // use timestamp of lidar measurement as reference:
  double cloud_time = cloud_data_buff_.front().time;
  // sync IMU, velocity and GNSS with lidar measurement:
  // find the two closest measurement around lidar measurement time
  // then use linear interpolation to generate synced measurement:
  bool valid_imu = sync_imu_data(unsynced_imu_, imu_data_buff_, cloud_time);
  bool valid_velocity = sync_velocity_data(unsynced_velocity_, velocity_data_buff_, cloud_time);
  bool valid_gnss = sync_gnss_data(unsynced_gnss_, gnss_data_buff_, cloud_time);

  // only mark lidar as 'inited' when all the three sensors are synced:
  static bool sensor_inited = false;
  if (!sensor_inited) {
    if (!valid_imu || !valid_velocity || !valid_gnss) {
      cloud_data_buff_.pop_front();
      return false;
    }
    sensor_inited = true;
  }

  return true;
}

bool DataPretreatNode::init_calibration()
{
  // lookup imu pose in lidar frame:
  static bool calibration_received = false;
  if (!calibration_received) {
    if (localization_common::lookup_in_tf_buffer(
        tf_buffer_, imu_frame_id_, lidar_frame_id_, lidar_to_imu_))
    {
      calibration_received = true;
    }
  }

  return calibration_received;
}

bool DataPretreatNode::init_gnss()
{
  static bool gnss_inited = false;
  if (!gnss_inited) {
    GNSSData gnss_data = gnss_data_buff_.front();
    gnss_data.init_origin_position();
    gnss_inited = true;
  }

  return gnss_inited;
}

bool DataPretreatNode::has_data()
{
  if (cloud_data_buff_.size() == 0) {
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

bool DataPretreatNode::valid_data()
{
  current_cloud_data_ = cloud_data_buff_.front();
  current_imu_data_ = imu_data_buff_.front();
  current_velocity_data_ = velocity_data_buff_.front();
  current_gnss_data_ = gnss_data_buff_.front();

  double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
  double diff_velocity_time = current_cloud_data_.time - current_velocity_data_.time;
  double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time;
  if (diff_imu_time < -0.05 || diff_velocity_time < -0.05 || diff_gnss_time < -0.05) {
    cloud_data_buff_.pop_front();
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

  cloud_data_buff_.pop_front();
  imu_data_buff_.pop_front();
  velocity_data_buff_.pop_front();
  gnss_data_buff_.pop_front();

  return true;
}

bool DataPretreatNode::transform_data()
{
  // a. get reference pose (GNSS & IMU pose prior):
  gnss_pose_ = Eigen::Matrix4f::Identity();
  // get position from GNSS
  current_gnss_data_.update_xyz();
  gnss_pose_(0, 3) = current_gnss_data_.local_E;
  gnss_pose_(1, 3) = current_gnss_data_.local_N;
  gnss_pose_(2, 3) = current_gnss_data_.local_U;
  // get orientation from IMU:
  gnss_pose_.block<3, 3>(0, 0) = current_imu_data_.orientation.matrix().cast<float>();
  // this is lidar pose in GNSS/map frame:
  gnss_pose_ *= lidar_to_imu_;

  // b. set synced pos vel
  pos_vel_.pos.x() = current_gnss_data_.local_E;
  pos_vel_.pos.y() = current_gnss_data_.local_N;
  pos_vel_.pos.z() = current_gnss_data_.local_U;
  pos_vel_.vel = current_velocity_data_.linear_velocity;

  // c. motion compensation for lidar measurements:
  current_velocity_data_.transform_coordinate(lidar_to_imu_);
  distortion_adjust_->set_motion_info(0.1, current_velocity_data_);
  distortion_adjust_->adjust_cloud(current_cloud_data_.cloud, current_cloud_data_.cloud);
  return true;
}

bool DataPretreatNode::publish_data()
{
  cloud_pub_->publish(current_cloud_data_.cloud, current_cloud_data_.time);
  imu_pub_->publish(current_imu_data_, current_cloud_data_.time);
  pos_vel_pub_->publish(pos_vel_, current_cloud_data_.time);
  //
  // this synced odometry has the following info:
  //
  // a. lidar frame's pose in map
  // b. lidar frame's velocity
  // gnss_pub_->publish(gnss_pose_, current_gnss_data_.time);
  gnss_pub_->publish(gnss_pose_, current_velocity_data_, current_cloud_data_.time);
  return true;
}
}  // namespace localization_common
