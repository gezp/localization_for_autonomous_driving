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

#include "localization_common/cloud_registration/ndt_registration.hpp"

namespace localization_common
{

NDTRegistration::NDTRegistration(const YAML::Node & node)
: ndt_(new pcl::NormalDistributionsTransform<PointXYZ, PointXYZ>())
{
  float res = node["res"].as<float>();
  float step_size = node["step_size"].as<float>();
  float trans_eps = node["trans_eps"].as<float>();
  int max_iter = node["max_iter"].as<int>();

  set_param(res, step_size, trans_eps, max_iter);
}

NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps, int max_iter)
: ndt_(new pcl::NormalDistributionsTransform<PointXYZ, PointXYZ>())
{
  set_param(res, step_size, trans_eps, max_iter);
}

bool NDTRegistration::set_param(float res, float step_size, float trans_eps, int max_iter)
{
  ndt_->setResolution(res);
  ndt_->setStepSize(step_size);
  ndt_->setTransformationEpsilon(trans_eps);
  ndt_->setMaximumIterations(max_iter);

  std::cout << "NDT params:" << std::endl
            << "res: " << res << ", "
            << "step_size: " << step_size << ", "
            << "trans_eps: " << trans_eps << ", "
            << "max_iter: " << max_iter << std::endl
            << std::endl;

  return true;
}

bool NDTRegistration::set_input_target(const PointXYZCloudPtr & input_target)
{
  ndt_->setInputTarget(input_target);

  return true;
}

bool NDTRegistration::match(
  const PointXYZCloudPtr & input_source, const Eigen::Matrix4f & predict_pose,
  PointXYZCloudPtr & result_cloud, Eigen::Matrix4f & result_pose)
{
  ndt_->setInputSource(input_source);
  ndt_->align(*result_cloud, predict_pose);
  result_pose = ndt_->getFinalTransformation();

  return true;
}
}  // namespace localization_common
