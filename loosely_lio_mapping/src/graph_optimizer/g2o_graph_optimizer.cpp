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

#include "loosely_lio_mapping/graph_optimizer/g2o_graph_optimizer.hpp"

#include "localization_common/tic_toc.hpp"

namespace loosely_lio_mapping
{
G2oGraphOptimizer::G2oGraphOptimizer(const YAML::Node & node)
{
  ImuPreIntegrationNoise noise;
  noise.prior = node["imu_noise"]["prior"].as<double>();
  noise.gyro = node["imu_noise"]["gyro"].as<double>();
  noise.accel = node["imu_noise"]["accel"].as<double>();
  noise.gyro_bias = node["imu_noise"]["gyro_bias"].as<double>();
  noise.accel_bias = node["imu_noise"]["accel_bias"].as<double>();
  std::cout << "G2oGraphOptimizer params:" << std::endl
            << "\tprior noise: " << noise.prior << std::endl
            << "\tprocess noise gyro: " << noise.gyro << std::endl
            << "\tprocess noise accel: " << noise.accel << std::endl
            << "\tprocess noise gyro_bias: " << noise.gyro_bias << std::endl
            << "\tprocess noise accel_bias: " << noise.accel_bias << std::endl
            << std::endl;
  // init imu pre-integration
  imu_pre_integration_ = std::make_shared<ImuPreIntegration>(noise);
  // init graph
  graph_.reset(new g2o::SparseOptimizer());
  g2o::OptimizationAlgorithmFactory * solver_factory =
    g2o::OptimizationAlgorithmFactory::instance();
  // solver_factory->listSolvers(std::cout);
  g2o::OptimizationAlgorithmProperty solver_property;
  std::string solver_type = "lm_var_csparse";
  g2o::OptimizationAlgorithm * solver = solver_factory->construct(solver_type, solver_property);
  graph_->setAlgorithm(solver);

  if (!graph_->solver()) {
    std::cerr << "Failed to create G2O optimizer!" << std::endl;
  }
  robust_kernel_factory_ = g2o::RobustKernelFactory::instance();
}

void G2oGraphOptimizer::add_vertex(
  const localization_common::ImuNavState & state, const bool need_fix)
{
  // init:
  g2o::VertexPRVAG * vertex(new g2o::VertexPRVAG());
  // a. set vertex ID:
  vertex->setId(graph_->vertices().size());
  // b. set vertex
  g2o::PRVAG measurement;
  measurement.time = state.time;
  measurement.pos = state.position;
  measurement.ori = Sophus::SO3d(Eigen::Quaterniond(state.orientation));
  // transform linear velocity from body frame to navigation frame:
  measurement.vel = state.linear_velocity;
  measurement.b_a = state.accel_bias;
  measurement.b_g = state.gyro_bias;
  vertex->setEstimate(measurement);
  // for first vertex:
  if (need_fix) {
    vertex->setFixed(true);
  }
  // add vertex:
  graph_->addVertex(vertex);
}

void G2oGraphOptimizer::add_relative_pose_edge(
  int v0, int v1, const Eigen::Matrix4d & relative_pose, const Eigen::VectorXd & noise)
{
  // init
  g2o::EdgePRVAGRelativePose * edge(new g2o::EdgePRVAGRelativePose());
  // set nodes
  edge->vertices()[0] = dynamic_cast<g2o::VertexPRVAG *>(graph_->vertex(v0));
  edge->vertices()[1] = dynamic_cast<g2o::VertexPRVAG *>(graph_->vertex(v1));
  // set measurement
  Eigen::Matrix<double, 6, 1> measurement = Eigen::Matrix<double, 6, 1>::Zero();
  // position
  measurement.block<3, 1>(g2o::EdgePRVAGRelativePose::INDEX_P, 0) = relative_pose.block<3, 1>(0, 3);
  // orientation, so3
  measurement.block<3, 1>(g2o::EdgePRVAGRelativePose::INDEX_R, 0) =
    Sophus::SO3d(Eigen::Quaterniond(relative_pose.block<3, 3>(0, 0))).log();
  edge->setMeasurement(measurement);
  // set information matrix
  Eigen::MatrixXd information_matrix = CalculateSe3EdgeInformationMatrix(noise);
  edge->setInformation(information_matrix);
  // set loss function
  if (need_robust_kernel_) {
    AddRobustKernel(edge, robust_kernel_name_, robust_kernel_size_);
  }
  // add edge
  graph_->addEdge(edge);
}

void G2oGraphOptimizer::add_prior_position_edge(
  int v0, const Eigen::Vector3d & pos, const Eigen::Vector3d & noise)
{
  // init:
  g2o::EdgePRVAGPriorPos * edge(new g2o::EdgePRVAGPriorPos());
  // set node:
  edge->vertices()[0] = dynamic_cast<g2o::VertexPRVAG *>(graph_->vertex(v0));
  // set measurement:
  edge->setMeasurement(pos);
  // set information matrix:
  Eigen::MatrixXd information_matrix = CalculateDiagMatrix(noise);
  edge->setInformation(information_matrix);
  // set loss function:
  if (need_robust_kernel_) {
    AddRobustKernel(edge, robust_kernel_name_, robust_kernel_size_);
  }
  // add edge:
  graph_->addEdge(edge);
}

void G2oGraphOptimizer::add_imu_data(const localization_common::IMUData & imu_data)
{
  imu_pre_integration_->integrate(imu_data);
}

void G2oGraphOptimizer::add_imu_pre_integration_edge(int v0, int v1)
{
  auto imu_pre_integration_state = imu_pre_integration_->get_state();
  // init
  g2o::EdgePRVAGIMUPreIntegration * edge(new g2o::EdgePRVAGIMUPreIntegration());
  // set nodes
  edge->vertices()[0] = dynamic_cast<g2o::VertexPRVAG *>(graph_->vertex(v0));
  edge->vertices()[1] = dynamic_cast<g2o::VertexPRVAG *>(graph_->vertex(v1));
  // set measurement
  edge->setT(imu_pre_integration_state.dt);
  edge->setGravitiy(gravity_);
  edge->setJacobian(imu_pre_integration_state.J);
  Eigen::Matrix<double, 15, 1> measurement = Eigen::Matrix<double, 15, 1>::Zero();
  measurement.block<3, 1>(0, 0) = imu_pre_integration_state.alpha_ij;
  Sophus::SO3d theta_ij(imu_pre_integration_state.theta_ij);
  measurement.block<3, 1>(3, 0) = theta_ij.log();
  measurement.block<3, 1>(6, 0) = imu_pre_integration_state.beta_ij;
  edge->setMeasurement(measurement);
  // set information matrix
  edge->setInformation(imu_pre_integration_state.P.inverse());
  // set loss function
  if (need_robust_kernel_) {
    AddRobustKernel(edge, robust_kernel_name_, robust_kernel_size_);
  }
  // add edge
  graph_->addEdge(edge);
  // reset imu pre-integration
  imu_pre_integration_->reset();
}

bool G2oGraphOptimizer::optimize()
{
  static int optimize_cnt = 0;
  if (graph_->edges().size() < 1) {
    return false;
  }
  std::cout << std::endl
            << "------ Begin Iteration " << ++optimize_cnt << " of Backend Optimization -------"
            << std::endl;

  localization_common::TicToc optimize_time;
  optimize_time.tic();
  graph_->initializeOptimization();
  graph_->computeInitialGuess();
  graph_->computeActiveErrors();
  graph_->setVerbose(false);

  double chi2 = graph_->chi2();
  int iterations = graph_->optimize(max_iterations_num_);

  std::cout << "Num. Vertices: " << graph_->vertices().size()
            << ", Num. Edges: " << graph_->edges().size() << std::endl
            << "Num. Iterations: " << iterations << "/" << max_iterations_num_ << std::endl
            << "Time Consumption: " << optimize_time.toc() << std::endl
            << "Cost Change: " << chi2 << "--->" << graph_->chi2() << std::endl
            << "------ Finish Backend Optimization -------" << std::endl
            << std::endl;

  return true;
}

int G2oGraphOptimizer::get_vertex_num() {return graph_->vertices().size();}

std::deque<localization_common::ImuNavState> G2oGraphOptimizer::get_optimized_vertices()
{
  std::deque<localization_common::ImuNavState> nav_states;
  const int N = graph_->vertices().size();

  for (int vertex_id = 0; vertex_id < N; vertex_id++) {
    g2o::VertexPRVAG * v = dynamic_cast<g2o::VertexPRVAG *>(graph_->vertex(vertex_id));
    const g2o::PRVAG & vertex_state = v->estimate();
    localization_common::ImuNavState state;
    // set state:
    state.time = vertex_state.time;
    state.position = vertex_state.pos;
    state.orientation = vertex_state.ori.matrix();
    state.linear_velocity = vertex_state.vel;
    state.accel_bias = vertex_state.b_a;
    state.gyro_bias = vertex_state.b_g;
    nav_states.push_back(state);
  }
  return nav_states;
}

void G2oGraphOptimizer::SetEdgeRobustKernel(
  std::string robust_kernel_name, double robust_kernel_size)
{
  robust_kernel_name_ = robust_kernel_name;
  robust_kernel_size_ = robust_kernel_size;
  need_robust_kernel_ = true;
}

Eigen::MatrixXd G2oGraphOptimizer::CalculateSe3EdgeInformationMatrix(Eigen::VectorXd noise)
{
  Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(6, 6);
  information_matrix = CalculateDiagMatrix(noise);
  return information_matrix;
}

void G2oGraphOptimizer::AddRobustKernel(
  g2o::OptimizableGraph::Edge * edge, const std::string & kernel_type, double kernel_size)
{
  if (kernel_type == "NONE") {
    return;
  }

  g2o::RobustKernel * kernel = robust_kernel_factory_->construct(kernel_type);
  if (kernel == nullptr) {
    std::cerr << "warning : invalid robust kernel type: " << kernel_type << std::endl;
    return;
  }

  kernel->setDelta(kernel_size);
  edge->setRobustKernel(kernel);
}

Eigen::MatrixXd G2oGraphOptimizer::CalculateDiagMatrix(Eigen::VectorXd noise)
{
  Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(noise.rows(), noise.rows());
  for (int i = 0; i < noise.rows(); i++) {
    information_matrix(i, i) /= noise(i);
  }
  return information_matrix;
}

}  // namespace loosely_lio_mapping
