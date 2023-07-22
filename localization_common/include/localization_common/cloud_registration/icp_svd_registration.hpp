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

#pragma once

#include <pcl/kdtree/kdtree_flann.h>
#include <yaml-cpp/yaml.h>

#include <vector>

#include "localization_common/cloud_registration/cloud_registration_interface.hpp"

namespace localization_common
{

class ICPSVDRegistration : public CloudRegistrationInterface
{
public:
  explicit ICPSVDRegistration(const YAML::Node & node);
  ICPSVDRegistration(float max_corr_dist, float trans_eps, float euc_fitness_eps, int max_iter);

  bool set_input_target(const PointXYZCloudPtr & input_target) override;
  bool match(
    const PointXYZCloudPtr & input_source, const Eigen::Matrix4f & predict_pose,
    PointXYZCloudPtr & result_cloud, Eigen::Matrix4f & result_pose) override;
  float get_fitness_score() override {return 0;}

private:
  bool set_param(float max_corr_dist, float trans_eps, float euc_fitness_eps, int max_iter);

private:
  size_t get_correspondence(
    const PointXYZCloudPtr & input_source, std::vector<Eigen::Vector3f> & xs,
    std::vector<Eigen::Vector3f> & ys);

  void get_transform(
    const std::vector<Eigen::Vector3f> & xs, const std::vector<Eigen::Vector3f> & ys,
    Eigen::Matrix4f & transformation);

  bool is_significant(const Eigen::Matrix4f & transformation, const float trans_eps);

  float max_corr_dist_;
  float trans_eps_;
  float euc_fitness_eps_;
  int max_iter_;

  PointXYZCloudPtr input_target_;
  pcl::KdTreeFLANN<PointXYZ>::Ptr input_target_kdtree_;
  PointXYZCloudPtr input_source_;

  Eigen::Matrix4f transformation_;
};

}  // namespace localization_common
