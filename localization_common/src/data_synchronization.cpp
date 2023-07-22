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

#include "localization_common/data_synchronization.hpp"

namespace localization_common
{

bool sync_gnss_data(
  std::deque<GnssData> & unsynced_data, std::deque<GnssData> & synced_data, double sync_time)
{
  // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
  // 即找到与同步时间相邻的左右两个数据
  // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
  while (unsynced_data.size() >= 2) {
    if (unsynced_data.front().time > sync_time) {
      return false;
    }
    if (unsynced_data.at(1).time < sync_time) {
      unsynced_data.pop_front();
      continue;
    }
    if (sync_time - unsynced_data.front().time > 0.2) {
      unsynced_data.pop_front();
      return false;
    }
    if (unsynced_data.at(1).time - sync_time > 0.2) {
      return false;
    }
    break;
  }
  if (unsynced_data.size() < 2) {
    return false;
  }

  GnssData front_data = unsynced_data.at(0);
  GnssData back_data = unsynced_data.at(1);
  GnssData data;

  double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
  double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
  data.time = sync_time;
  data.status = back_data.status;
  data.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;
  data.latitude = front_data.latitude * front_scale + back_data.latitude * back_scale;
  data.altitude = front_data.altitude * front_scale + back_data.altitude * back_scale;
  data.local_E = front_data.local_E * front_scale + back_data.local_E * back_scale;
  data.local_N = front_data.local_N * front_scale + back_data.local_N * back_scale;
  data.local_U = front_data.local_U * front_scale + back_data.local_U * back_scale;

  synced_data.push_back(data);

  return true;
}
bool sync_imu_data2(
  std::deque<ImuData2> & unsynced_data, std::deque<ImuData2> & synced_data, double sync_time)
{
  // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
  // 即找到与同步时间相邻的左右两个数据
  // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
  while (unsynced_data.size() >= 2) {
    // unsynced_data.front().time should be <= sync_time:
    if (unsynced_data.front().time > sync_time) {
      return false;
    }
    // sync_time should be <= unsynced_data.at(1).time:
    if (unsynced_data.at(1).time < sync_time) {
      unsynced_data.pop_front();
      continue;
    }

    // sync_time - unsynced_data.front().time should be <= 0.2:
    if (sync_time - unsynced_data.front().time > 0.2) {
      unsynced_data.pop_front();
      return false;
    }
    // unsynced_data.at(1).time - sync_time should be <= 0.2
    if (unsynced_data.at(1).time - sync_time > 0.2) {
      return false;
    }
    break;
  }
  if (unsynced_data.size() < 2) {
    return false;
  }

  ImuData2 front_data = unsynced_data.at(0);
  ImuData2 back_data = unsynced_data.at(1);
  ImuData2 data;

  double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
  double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
  data.time = sync_time;
  auto v_x = front_data.linear_acceleration.x() * front_scale +
    back_data.linear_acceleration.x() * back_scale;
  auto v_y = front_data.linear_acceleration.y() * front_scale +
    back_data.linear_acceleration.y() * back_scale;
  auto v_z = front_data.linear_acceleration.z() * front_scale +
    back_data.linear_acceleration.z() * back_scale;
  data.linear_acceleration = Eigen::Vector3d(v_x, v_y, v_z);
  auto w_x =
    front_data.angular_velocity.x() * front_scale + back_data.angular_velocity.x() * back_scale;
  auto w_y =
    front_data.angular_velocity.y() * front_scale + back_data.angular_velocity.y() * back_scale;
  auto w_z =
    front_data.angular_velocity.z() * front_scale + back_data.angular_velocity.z() * back_scale;
  data.angular_velocity = Eigen::Vector3d(w_x, w_y, w_z);

  // 四元数插值有线性插值和球面插值，球面插值更准确，但是两个四元数差别不大是，二者精度相当
  // 由于是对相邻两时刻姿态插值，姿态差比较小，所以可以用线性插值
  auto q_x = front_data.orientation.x() * front_scale + back_data.orientation.x() * back_scale;
  auto q_y = front_data.orientation.y() * front_scale + back_data.orientation.y() * back_scale;
  auto q_z = front_data.orientation.z() * front_scale + back_data.orientation.z() * back_scale;
  auto q_w = front_data.orientation.w() * front_scale + back_data.orientation.w() * back_scale;
  // 线性插值之后要归一化
  data.orientation = Eigen::Quaterniond(q_w, q_x, q_y, q_z);
  data.orientation.normalize();

  synced_data.push_back(data);

  return true;
}
bool sync_velocity_data(
  std::deque<VelocityData> & unsynced_data, std::deque<VelocityData> & synced_data,
  double sync_time)
{
  // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
  // 即找到与同步时间相邻的左右两个数据
  // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
  while (unsynced_data.size() >= 2) {
    if (unsynced_data.front().time > sync_time) {
      return false;
    }
    if (unsynced_data.at(1).time < sync_time) {
      unsynced_data.pop_front();
      continue;
    }
    if (sync_time - unsynced_data.front().time > 0.2) {
      unsynced_data.pop_front();
      return false;
    }
    if (unsynced_data.at(1).time - sync_time > 0.2) {
      unsynced_data.pop_front();
      return false;
    }
    break;
  }
  if (unsynced_data.size() < 2) {
    return false;
  }

  VelocityData front_data = unsynced_data.at(0);
  VelocityData back_data = unsynced_data.at(1);
  VelocityData data;

  double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
  double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
  data.time = sync_time;
  auto v_x =
    front_data.linear_velocity.x() * front_scale + back_data.linear_velocity.x() * back_scale;
  auto v_y =
    front_data.linear_velocity.y() * front_scale + back_data.linear_velocity.y() * back_scale;
  auto v_z =
    front_data.linear_velocity.z() * front_scale + back_data.linear_velocity.z() * back_scale;
  data.linear_velocity = Eigen::Vector3f(v_x, v_y, v_z);
  auto w_x =
    front_data.angular_velocity.x() * front_scale + back_data.angular_velocity.x() * back_scale;
  auto w_y =
    front_data.angular_velocity.y() * front_scale + back_data.angular_velocity.y() * back_scale;
  auto w_z =
    front_data.angular_velocity.z() * front_scale + back_data.angular_velocity.z() * back_scale;
  data.angular_velocity = Eigen::Vector3f(w_x, w_y, w_z);
  synced_data.push_back(data);

  return true;
}

}  // namespace localization_common
