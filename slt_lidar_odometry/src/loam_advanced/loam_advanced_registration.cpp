// Copyright 2026 Gezp (https://github.com/gezp).
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

#include "slt_lidar_odometry/loam_advanced/loam_advanced_registration.hpp"

#include <pcl/common/transforms.h>

#include "slt_lidar_odometry/loam_advanced/loam_advanced_factor.hpp"


namespace slt_lidar_odometry
{

using EdgeCorrespondence = LoamAdvancedRegistration::EdgeCorrespondence;
using SurfCorrespondence = LoamAdvancedRegistration::SurfCorrespondence;

LoamAdvancedRegistration::LoamAdvancedRegistration(const YAML::Node & config)
{
  num_nearby_ = config["num_nearby"].as<int>();
  double nearby_distance = config["nearby_distance"].as<double>();
  sqr_nearby_distance_ = nearby_distance * nearby_distance;
  edge_eigen_ratio_ = config["edge_eigen_ratio"].as<double>();
  max_line_distance_ = config["max_line_distance"].as<double>();
  max_plane_distance_ = config["max_plane_distance"].as<double>();
  num_optimization_ = config["num_optimization"].as<int>();
  use_analytic_derivatives_ = config["use_analytic_derivatives"].as<bool>();
  ceres_loss_ = config["ceres_loss"].as<double>();
  max_num_iterations_ = config["max_num_iterations"].as<int>();
  min_num_residual_blocks_ = config["min_num_residual_blocks"].as<int>();
  minimizer_progress_to_stdout_ = config["minimizer_progress_to_stdout"].as<bool>();
  debug_ = config["debug"].as<bool>();
  bool enabel = config["enable_elapsed_time_statistics"].as<bool>();
  elapsed_time_statistics_.set_enable(enabel);
  elapsed_time_statistics_.set_title("LoamAdvancedRegistration");
}

bool LoamAdvancedRegistration::set_target(const LoamAdvancedFeature & target)
{
  edge_map_ = target.edge;
  surf_map_ = target.surf;
  edge_kd_tree_.reset(new pcl::KdTreeFLANN<PointType>);
  surf_kd_tree_.reset(new pcl::KdTreeFLANN<PointType>);
  if (edge_map_ && edge_map_->size() > 0) {
    edge_kd_tree_->setInputCloud(edge_map_);
  }
  if (surf_map_ && surf_map_->size() > 0) {
    surf_kd_tree_->setInputCloud(surf_map_);
  }
  return true;
}

bool LoamAdvancedRegistration::match(
  const LoamAdvancedFeature & input, const Eigen::Matrix4d & initial_pose)
{
  elapsed_time_statistics_.tic("match");
  // initialize ceres parameters
  Eigen::Map<Eigen::Quaterniond> q(ceres_parameter_);
  Eigen::Map<Eigen::Vector3d> t(ceres_parameter_ + 4);
  q = Eigen::Quaterniond(initial_pose.block<3, 3>(0, 0));
  t = initial_pose.block<3, 1>(0, 3);
  // optimize
  for (int i = 0; i < num_optimization_; i++) {
    ceres::Problem problem;
    problem.AddParameterBlock(ceres_parameter_, 4);
    problem.AddParameterBlock(ceres_parameter_ + 4, 3);
    // analytic factors give tangent jacobians, pair with SO3Manifold
    // (identity PlusJacobian); autodiff gives ambient jacobians, pair with
    // EigenQuaternionManifold
    ceres::Manifold * q_manifold;
    if (use_analytic_derivatives_) {
      q_manifold = new SO3Manifold();
    } else {
      q_manifold = new ceres::EigenQuaternionManifold();
    }
    problem.SetManifold(ceres_parameter_, q_manifold);
    ceres::LossFunction * loss_function = new ceres::HuberLoss(ceres_loss_);
    // find correspondence for edge features
    elapsed_time_statistics_.tic("find edge correspondence");
    auto edge_infos = find_all_edge_correspondence(*input.edge, get_final_pose());
    // add residual blocks
    for (auto & info : edge_infos) {
      ceres::CostFunction * cost_function;
      if (use_analytic_derivatives_) {
        cost_function = new LoamAdvancedEdgeAnalyticFactor(
          info.current_point, info.line_point, info.line_direction);
      } else {
        cost_function = LoamAdvancedEdgeFactor::create(
          info.current_point, info.line_point, info.line_direction);
      }
      problem.AddResidualBlock(
        cost_function, loss_function, ceres_parameter_, ceres_parameter_ + 4);
    }
    elapsed_time_statistics_.toc("find edge correspondence");
    // find correspondence for surf features
    elapsed_time_statistics_.tic("find surf correspondence");
    auto surf_infos = find_all_surf_correspondence(*input.surf, get_final_pose());
    // add residual blocks
    for (auto & info : surf_infos) {
      ceres::CostFunction * cost_function;
      if (use_analytic_derivatives_) {
        cost_function = new LoamAdvancedSurfAnalyticFactor(
          info.current_point, info.plane_point, info.plane_normal);
      } else {
        cost_function = LoamAdvancedSurfFactor::create(
          info.current_point, info.plane_point, info.plane_normal);
      }
      problem.AddResidualBlock(
        cost_function, loss_function, ceres_parameter_, ceres_parameter_ + 4);
    }
    elapsed_time_statistics_.toc("find surf correspondence");
    if (debug_) {
      std::cout << "optimization [" << i + 1 << "] edge points: " << edge_infos.size()
                << ", surf points: " << surf_infos.size() << std::endl;
    }
    if (problem.NumResidualBlocks() < min_num_residual_blocks_) {
      std::cout << "the num of residual blocks are too small: " << problem.NumResidualBlocks()
                << std::endl;
      elapsed_time_statistics_.toc("match");
      return false;
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = max_num_iterations_;
    options.minimizer_progress_to_stdout = minimizer_progress_to_stdout_;
    ceres::Solver::Summary summary;
    elapsed_time_statistics_.tic("optimaztion");
    ceres::Solve(options, &problem, &summary);
    elapsed_time_statistics_.toc("optimaztion");
  }
  elapsed_time_statistics_.toc("match");
  elapsed_time_statistics_.print_all_info("match", 20);
  return true;
}

Eigen::Matrix4d LoamAdvancedRegistration::get_final_pose()
{
  Eigen::Map<Eigen::Quaterniond> q(ceres_parameter_);
  Eigen::Map<Eigen::Vector3d> t(ceres_parameter_ + 4);
  Eigen::Matrix4d final_pose = Eigen::Matrix4d::Identity();
  q.normalize();
  final_pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  final_pose.block<3, 1>(0, 3) = t;
  return final_pose;
}

bool LoamAdvancedRegistration::fit_line(
  const PointCloudType & point_cloud, const std::vector<int> & indices, Eigen::Vector3d & p0,
  Eigen::Vector3d & d)
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (auto idx : indices) {
    center += point_cloud.points[idx].getVector3fMap().cast<double>();
  }
  center /= static_cast<double>(indices.size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (auto idx : indices) {
    Eigen::Vector3d p = point_cloud.points[idx].getVector3fMap().cast<double>() - center;
    covariance += p * p.transpose();
  }
  covariance /= static_cast<double>(indices.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  // line: largest eigen vector, check shape by eigen value ratio
  d = solver.eigenvectors().col(2);
  double largest = solver.eigenvalues()(2);
  double middle = solver.eigenvalues()(1);
  if (middle <= std::numeric_limits<double>::epsilon() ||
      largest < edge_eigen_ratio_ * middle) {
    return false;
  }
  p0 = center;
  return true;
}

bool LoamAdvancedRegistration::fit_plane(
  const PointCloudType & point_cloud, const std::vector<int> & indices, Eigen::Vector3d & p0,
  Eigen::Vector3d & n)
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (auto idx : indices) {
    center += point_cloud.points[idx].getVector3fMap().cast<double>();
  }
  center /= static_cast<double>(indices.size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (auto idx : indices) {
    Eigen::Vector3d p = point_cloud.points[idx].getVector3fMap().cast<double>() - center;
    covariance += p * p.transpose();
  }
  covariance /= static_cast<double>(indices.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  // plane: smallest eigen vector, check flatness by eigen value ratio
  n = solver.eigenvectors().col(0);
  double smallest = solver.eigenvalues()(0);
  double middle = solver.eigenvalues()(1);
  if (smallest <= std::numeric_limits<double>::epsilon() ||
      middle < edge_eigen_ratio_ * smallest) {
    return false;
  }
  p0 = center;
  return true;
}

std::vector<EdgeCorrespondence> LoamAdvancedRegistration::find_all_edge_correspondence(
  const PointCloudType & point_cloud, const Eigen::Matrix4d & initial_pose)
{
  Eigen::Affine3f T_map_current(initial_pose.cast<float>());
  std::vector<EdgeCorrespondence> result;
  if (!edge_kd_tree_ || edge_kd_tree_->getInputCloud() == nullptr) {
    return result;
  }
  std::vector<int> search_indices;
  std::vector<float> search_sqr_distances;
  for (size_t i = 0; i < point_cloud.points.size(); ++i) {
    // transform point to map frame
    auto p = pcl::transformPoint(point_cloud.points[i], T_map_current);
    edge_kd_tree_->nearestKSearch(p, num_nearby_, search_indices, search_sqr_distances);
    if (static_cast<int>(search_indices.size()) < num_nearby_) {
      continue;
    }
    // skip if the distance of farthest neighbor is larger than threshold
    if (search_sqr_distances.back() > sqr_nearby_distance_) {
      continue;
    }
    // fit line by neighbors
    Eigen::Vector3d line_point;
    Eigen::Vector3d line_direction;
    if (!fit_line(*edge_map_, search_indices, line_point, line_direction)) {
      continue;
    }
    // skip if distance to line is larger than threshold
    Eigen::Vector3d p_map = p.getVector3fMap().cast<double>();
    Eigen::Vector3d diff = p_map - line_point;
    double line_distance = (diff - line_direction * line_direction.dot(diff)).norm();
    if (line_distance > max_line_distance_) {
      continue;
    }
    EdgeCorrespondence info;
    info.current_point = point_cloud.points[i].getVector3fMap().cast<double>();
    info.line_point = line_point;
    info.line_direction = line_direction;
    result.push_back(std::move(info));
  }
  return result;
}

std::vector<SurfCorrespondence> LoamAdvancedRegistration::find_all_surf_correspondence(
  const PointCloudType & point_cloud, const Eigen::Matrix4d & initial_pose)
{
  Eigen::Affine3f T_map_current(initial_pose.cast<float>());
  std::vector<SurfCorrespondence> result;
  if (!surf_kd_tree_ || surf_kd_tree_->getInputCloud() == nullptr) {
    return result;
  }
  std::vector<int> search_indices;
  std::vector<float> search_sqr_distances;
  for (size_t i = 0; i < point_cloud.points.size(); ++i) {
    // transform point to map frame
    auto p = pcl::transformPoint(point_cloud.points[i], T_map_current);
    surf_kd_tree_->nearestKSearch(p, num_nearby_, search_indices, search_sqr_distances);
    if (static_cast<int>(search_indices.size()) < num_nearby_) {
      continue;
    }
    if (search_sqr_distances.back() > sqr_nearby_distance_) {
      continue;
    }
    // fit plane by neighbors
    Eigen::Vector3d plane_point;
    Eigen::Vector3d plane_normal;
    if (!fit_plane(*surf_map_, search_indices, plane_point, plane_normal)) {
      continue;
    }
    // skip if distance to plane is larger than threshold
    Eigen::Vector3d p_map = p.getVector3fMap().cast<double>();
    double plane_distance = std::abs((p_map - plane_point).dot(plane_normal));
    if (plane_distance > max_plane_distance_) {
      continue;
    }
    SurfCorrespondence info;
    info.current_point = point_cloud.points[i].getVector3fMap().cast<double>();
    info.plane_point = plane_point;
    info.plane_normal = plane_normal;
    result.push_back(std::move(info));
  }
  return result;
}

}  // namespace slt_lidar_odometry
