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

#include "lidar_mapping/front_end_node.hpp"

#include <filesystem>

namespace lidar_mapping
{
FrontEndNode::FrontEndNode(rclcpp::Node::SharedPtr node)
{
  std::string front_end_config;
  node->declare_parameter("front_end_config", front_end_config);
  node->get_parameter("front_end_config", front_end_config);
  RCLCPP_INFO(node->get_logger(), "front_end_config: [%s]", front_end_config.c_str());
  if (front_end_config == "" || (!std::filesystem::exists(front_end_config))) {
    RCLCPP_FATAL(node->get_logger(), "front_end_config is invalid");
    return;
  }
  front_end_ = std::make_shared<FrontEnd>();
  std::cout << "-----------------Init Lidar Frontend-------------------" << std::endl;
  front_end_->init_config(front_end_config);
  // sub & pub
  cloud_sub_ = std::make_shared<localization_common::CloudSubscriber>(node, "synced_cloud", 100000);
  lidar_odom_pub_ = std::make_shared<localization_common::OdometryPublisher>(
    node, "lidar_odom", "map", "lidar", 100);
  // process loop flow callback
  run_thread_ = std::make_unique<std::thread>(
    [this]() {
      while (!exit_) {
        run();
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(10ms);
      }
    });
}

FrontEndNode::~FrontEndNode()
{
  exit_ = true;
  if (run_thread_) {
    run_thread_->join();
  }
}

bool FrontEndNode::run()
{
  if (!read_data()) {
    return false;
  }
  while (has_data()) {
    if (!valid_data()) {
      continue;
    }
    if (update_odometry()) {
      publish_data();
    }
  }
  return true;
}

bool FrontEndNode::read_data()
{
  cloud_sub_->parse_data(cloud_data_buff_);
  return true;
}

bool FrontEndNode::has_data() {return cloud_data_buff_.size() > 0;}

bool FrontEndNode::valid_data()
{
  current_cloud_data_ = cloud_data_buff_.front();
  cloud_data_buff_.pop_front();

  return true;
}

bool FrontEndNode::update_odometry()
{
  static bool odometry_inited = false;
  if (!odometry_inited) {
    odometry_inited = true;
    // init lidar odometry:
    front_end_->set_init_pose(Eigen::Matrix4f::Identity());
  }
  // update lidar odometry using current undistorted measurement:
  return front_end_->update(current_cloud_data_, lidar_odometry_);
}

bool FrontEndNode::publish_data()
{
  lidar_odom_pub_->publish(lidar_odometry_, current_cloud_data_.time);
  return true;
}
}  // namespace lidar_mapping
