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

#include "localization_common/cloud_registration/icp_registration.hpp"

namespace localization_common
{

ICPRegistration::ICPRegistration(const YAML::Node & node)
: icp_(new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>())
{
  float max_corr_dist = node["max_corr_dist"].as<float>();
  float trans_eps = node["trans_eps"].as<float>();
  float euc_fitness_eps = node["euc_fitness_eps"].as<float>();
  int max_iter = node["max_iter"].as<int>();

  set_param(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
}

ICPRegistration::ICPRegistration(
  float max_corr_dist, float trans_eps, float euc_fitness_eps, int max_iter)
: icp_(new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>())
{
  set_param(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
}

bool ICPRegistration::set_param(
  float max_corr_dist, float trans_eps, float euc_fitness_eps, int max_iter)
{
  icp_->setMaxCorrespondenceDistance(max_corr_dist);
  icp_->setTransformationEpsilon(trans_eps);
  icp_->setEuclideanFitnessEpsilon(euc_fitness_eps);
  icp_->setMaximumIterations(max_iter);
  return true;
}

bool ICPRegistration::set_target(const PointCloudPtr & target)
{
  icp_->setInputTarget(target);
  return true;
}

bool ICPRegistration::match(
  const ICPRegistration::PointCloudPtr & input, const Eigen::Matrix4f & initial_pose)
{
  PointCloudPtr result_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  icp_->setInputSource(input);
  icp_->align(*result_cloud, initial_pose);
  return true;
}
Eigen::Matrix4f ICPRegistration::get_final_pose() {return icp_->getFinalTransformation();}

double ICPRegistration::get_fitness_score() {return icp_->getFitnessScore();}

void ICPRegistration::print_info()
{
  std::cout << "[ICP] "
            << "max_corr_dist: " << icp_->getMaxCorrespondenceDistance() << ", "
            << "trans_eps: " << icp_->getTransformationEpsilon() << ", "
            << "euc_fitness_eps: " << icp_->getEuclideanFitnessEpsilon() << ", "
            << "max_iter: " << icp_->getMaximumIterations() << std::endl;
}

}  // namespace localization_common
