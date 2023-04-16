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

#include "localization_common/subscriber/gnss_subscriber.hpp"

namespace localization_common
{
GNSSSubscriber::GNSSSubscriber(
  rclcpp::Node::SharedPtr node, std::string topic_name, size_t buff_size)
: node_(node)
{
  subscriber_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
    topic_name, buff_size, std::bind(&GNSSSubscriber::msg_callback, this, std::placeholders::_1));
}

void GNSSSubscriber::set_gnss_datum(double latitude, double longitude, double altitude)
{
  geo_converter_.Reset(latitude, longitude, altitude);
  origin_position_inited_ = true;
  RCLCPP_INFO(
    node_->get_logger(), "Set gnss local cartesian datum: (%lf, %lf, %lf)", latitude, longitude,
    altitude);
}

void GNSSSubscriber::msg_callback(const sensor_msgs::msg::NavSatFix::SharedPtr nav_sat_fix_ptr)
{
  GNSSData gnss_data;
  gnss_data.time = rclcpp::Time(nav_sat_fix_ptr->header.stamp).seconds();
  gnss_data.latitude = nav_sat_fix_ptr->latitude;
  gnss_data.longitude = nav_sat_fix_ptr->longitude;
  gnss_data.altitude = nav_sat_fix_ptr->altitude;
  gnss_data.status = nav_sat_fix_ptr->status.status;
  gnss_data.service = nav_sat_fix_ptr->status.service;
  // convert gnss
  if (!origin_position_inited_) {
    geo_converter_.Reset(gnss_data.latitude, gnss_data.longitude, gnss_data.altitude);
    origin_position_inited_ = true;
    RCLCPP_INFO(
      node_->get_logger(), "Use the first gnss data as gnss datum: (%lf, %lf, %lf)",
      gnss_data.latitude, gnss_data.longitude, gnss_data.altitude);
  }
  geo_converter_.Forward(
    gnss_data.latitude, gnss_data.longitude, gnss_data.altitude, gnss_data.local_E,
    gnss_data.local_N, gnss_data.local_U);
  // push to queue
  buff_mutex_.lock();
  new_gnss_data_.push_back(gnss_data);
  buff_mutex_.unlock();
}

void GNSSSubscriber::parse_data(std::deque<GNSSData> & gnss_data_buff)
{
  buff_mutex_.lock();
  if (new_gnss_data_.size() > 0) {
    gnss_data_buff.insert(gnss_data_buff.end(), new_gnss_data_.begin(), new_gnss_data_.end());
    new_gnss_data_.clear();
  }
  buff_mutex_.unlock();
}

}  // namespace localization_common
