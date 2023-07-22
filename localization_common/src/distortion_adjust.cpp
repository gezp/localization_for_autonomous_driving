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

#include "localization_common/distortion_adjust.hpp"

#include <mutex>
#include <thread>

#include "localization_common/tic_toc.hpp"

namespace localization_common
{
void DistortionAdjust::set_motion_info(float scan_period, VelocityData velocity_data)
{
  scan_period_ = scan_period;
  velocity_ = velocity_data.linear_velocity;
  angular_rate_ = velocity_data.angular_velocity;
}

bool DistortionAdjust::adjust_cloud(
  pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & output_cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloud;
  origin_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>(*input_cloud));

  float orientation_space = 2.0 * M_PI;
  float delete_space = 5.0 * M_PI / 180.0;
  float start_orientation = atan2(origin_cloud->points[0].y, origin_cloud->points[0].x);

  Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f::UnitZ());
  Eigen::Matrix3f rotate_matrix = t_V.matrix();
  Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
  transform_matrix.block<3, 3>(0, 0) = rotate_matrix.inverse();
  pcl::transformPointCloud(*origin_cloud, *origin_cloud, transform_matrix);

  velocity_ = rotate_matrix * velocity_;
  angular_rate_ = rotate_matrix * angular_rate_;
  // adjust for each point
  output_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
  // TicToc timer
  for (size_t point_index = 1; point_index < origin_cloud->points.size(); ++point_index) {
    auto & origin_p = origin_cloud->points[point_index];
    float orientation = atan2(origin_p.y, origin_p.x);
    if (orientation < 0.0) {
      orientation += 2.0 * M_PI;
    }
    if (orientation < delete_space || 2.0 * M_PI - orientation < delete_space) {
      continue;
    }
    float real_time = fabs(orientation) / orientation_space * scan_period_ - scan_period_ / 2.0;
    Eigen::Vector3f origin_point(origin_p.x, origin_p.y, origin_p.z);
    Eigen::Matrix3f current_matrix = update_matrix(real_time);
    // Rp + t
    Eigen::Vector3f adjusted_point = current_matrix * origin_point + velocity_ * real_time;
    pcl::PointXYZ point(adjusted_point.x(), adjusted_point.y(), adjusted_point.z());
    output_cloud->points.push_back(point);
  }
  // std::cout<< "adjust_cloud Time Consumption: " << timer.toc() << std::endl;
  pcl::transformPointCloud(*output_cloud, *output_cloud, transform_matrix.inverse());
  return true;
}

template<typename T>
Eigen::Quaternion<T> AngleAxisVectorToRotationQuaternion(const Eigen::Matrix<T, 3, 1> & angle_axis)
{
  T scale = T(0.5);
  T w = T(1.);
  constexpr double kCutoffAngle = 1e-8;  // We linearize below this angle.
  if (angle_axis.squaredNorm() > kCutoffAngle) {
    const T norm = angle_axis.norm();
    scale = sin(norm / 2.) / norm;
    w = cos(norm / 2.);
  }
  const Eigen::Matrix<T, 3, 1> quaternion_xyz = scale * angle_axis;
  return Eigen::Quaternion<T>(w, quaternion_xyz.x(), quaternion_xyz.y(), quaternion_xyz.z());
}

Eigen::Matrix3f DistortionAdjust::update_matrix(float real_time)
{
  Eigen::Vector3f angle = angular_rate_ * real_time;
  // Eigen::AngleAxisf t_Vz(angle(2), Eigen::Vector3f::UnitZ());
  // Eigen::AngleAxisf t_Vy(angle(1), Eigen::Vector3f::UnitY());
  // Eigen::AngleAxisf t_Vx(angle(0), Eigen::Vector3f::UnitX());
  // Eigen::AngleAxisf t_V;
  // t_V = t_Vz * t_Vy * t_Vx;
  // return t_V.matrix();
  return AngleAxisVectorToRotationQuaternion(angle).toRotationMatrix();
}
}  // namespace localization_common
