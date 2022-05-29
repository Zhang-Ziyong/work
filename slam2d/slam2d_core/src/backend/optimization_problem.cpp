#include "backend/optimization_problem.hpp"
#include "backend/spa_cost_function_2d.hpp"
#include "backend/gyro_pre_integration.hpp"
#include "backend/rotation_cost_function_2d.hpp"

namespace slam2d_core {
namespace backend {
namespace {

std::array<double, 3> fromPose(const common::Rigid2 &pose) {
  return {
      {pose.translation().x(), pose.translation().y(), pose.normalizedAngle()}};
}

common::Rigid2 toPose(const std::array<double, 3> &values) {
  return common::Rigid2({values[0], values[1]}, values[2]);
}

template <typename IteratorType>
double InterpolateAngularVelocityYaw(const IteratorType &start,
                                     const IteratorType &end,
                                     const common::Time time) {
  CHECK_LE(start->first, time);
  CHECK_GE(end->first, time);

  const double duration = common::toSeconds(end->first - start->first);
  const double factor = common::toSeconds(time - start->first) / duration;
  return start->second.angular_velocity[2] +
         (end->second.angular_velocity[2] - start->second.angular_velocity[2]) *
             factor;
}

}  // namespace

OptimizationProblem::OptimizationProblem(
    const OptimizationProblemOptions &options)
    : options_(options) {}

OptimizationProblem::~OptimizationProblem() {}

void OptimizationProblem::addImuData(const common::ImuData &imu_data) {
  imu_data_.emplace(imu_data.time, imu_data);
}

void OptimizationProblem::addOdometryData(
    const common::OdometryData &odometry_data) {
  odometry_data_.emplace(odometry_data.time, odometry_data);
}

void OptimizationProblem::addTrajectoryNode(const NodeSpec2D &node_data) {
  node_data_.emplace(trajectory_data_.node_id, node_data);
  trajectory_data_.node_id++;
}

void OptimizationProblem::insertTrajectoryNode(const int &node_id,
                                               const NodeSpec2D &node_data) {
  node_data_.emplace(node_id, node_data);
  trajectory_data_.node_id++;
}

void OptimizationProblem::trimOdomtryData(const int &node_id) {
  const auto node_it = node_data_.find(node_id);
  CHECK(node_it != node_data_.end());

  const common::Time gap_start = node_it != node_data_.begin()
                                     ? std::prev(node_it)->second.time
                                     : common::Time::min();

  const auto next_it = std::next(node_it);
  const common::Time gap_end =
      next_it != node_data_.end() ? next_it->second.time : common::Time::max();

  if (gap_start > gap_end) {
    LOG(WARNING) << "gap_start > gap_end. " << gap_start << "---" << gap_end;
    return;
  }

  auto data_it = odometry_data_.lower_bound(gap_start);
  auto data_end = odometry_data_.upper_bound(gap_end);

  if (data_it == data_end) {
    return;
  }
  if (gap_end != common::Time::max()) {
    data_end = std::prev(data_end);
    if (data_it == data_end) {
      return;
    }
  }
  if (gap_start != common::Time::min()) {
    data_it = std::next(data_it);
  }
  while (data_it != data_end) { data_it = odometry_data_.erase(data_it); }
}

void OptimizationProblem::trimTrajectoryNode(const int &node_id) {
  trimOdomtryData(node_id);
  const auto it = node_data_.find(node_id);
  CHECK(it != node_data_.end());
  node_data_.erase(it);
}

void OptimizationProblem::addSubmap(const common::Rigid2 &global_submap_pose) {
  // LOG(INFO) << "add submap pose: " << global_submap_pose.translation().x()
  //           << ", " << global_submap_pose.translation().y();

  submap_data_.emplace(trajectory_data_.submap_id,
                       SubmapSpec2D{global_submap_pose});
  trajectory_data_.submap_id++;
}

void OptimizationProblem::insertSubmap(
    const int &submap_id, const common::Rigid2 &global_submap_pose) {
  submap_data_.emplace(submap_id, SubmapSpec2D{global_submap_pose});
  trajectory_data_.submap_id++;
}

void OptimizationProblem::trimSubmap(const int &submap_id) {
  const auto it = submap_data_.find(submap_id);
  CHECK(it != submap_data_.end());
  submap_data_.erase(it);
}

void OptimizationProblem::setMaxNumIterations(
    const int32_t max_num_iterations) {
  options_.ceres_solver_options.max_num_iterations = max_num_iterations;
}

void OptimizationProblem::solve(const std::vector<Constraint> &constraints) {
  if (node_data_.empty()) {
    // Nothing to optimize.
    return;
  }

  if (!imu_data_.empty()) {
    size_t segment_num = 1 + node_data_.size() / options_.num_node_one_segment;

    // initialize segment gyro bias using last gyro bias or prior
    while (segment_gyr_bias_data_.size() < segment_num) {
      int segment_id = segment_gyr_bias_data_.size();
      if (!segment_gyr_bias_data_.empty()) {
        segment_gyr_bias_data_.emplace(
            segment_id, std::prev(segment_gyr_bias_data_.end())->second);
      } else {
        GyroBiasData gyro_bias_prior;
        gyro_bias_prior.gyro_bias[2] = 0.;
        segment_gyr_bias_data_.emplace(segment_id, gyro_bias_prior);
      }
    }
  }

  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);

  std::map<int, std::array<double, 3>> C_submaps;       // int -> submap id
  std::map<int, std::array<double, 3>> C_nodes;         // int -> node id
  std::map<int, std::array<double, 1>> C_segment_bg_z;  // int -> segment id

  bool first_submap = true;
  for (const auto &submap_id_data : submap_data_) {
    int submap_id = submap_id_data.first;
    C_submaps.emplace(submap_id, fromPose(submap_id_data.second.global_pose));

    problem.AddParameterBlock(C_submaps.at(submap_id).data(), 3);

    if (first_submap) {
      first_submap = false;
      problem.SetParameterBlockConstant(C_submaps.at(submap_id).data());
    }
  }

  for (const auto &node_id_data : node_data_) {
    int node_id = node_id_data.first;
    C_nodes.emplace(node_id, fromPose(node_id_data.second.global_pose_2d));
    problem.AddParameterBlock(C_nodes.at(node_id).data(), 3);
  }

  for (const auto &segment_gyr_bias : segment_gyr_bias_data_) {
    int segment_id = segment_gyr_bias.first;
    std::array<double, 1> gyro_bias_z = {
        {segment_gyr_bias.second.gyro_bias[2]}};
    C_segment_bg_z.emplace(segment_id, gyro_bias_z);
    problem.AddParameterBlock(C_segment_bg_z.at(segment_id).data(), 1);
  }

  { problem.AddParameterBlock(td_data_.lidar_to_gyroscope_td.data(), 1); }

  for (const Constraint &constraint : constraints) {
    problem.AddResidualBlock(
        createAutoDiffSpaCostFunction(constraint.pose),
        // Loop closure constraints should have a loss function.
        constraint.tag == Constraint::INTER_SUBMAP
            ? new ceres::HuberLoss(options_.huber_scale)
            : nullptr,
        C_submaps.at(constraint.submap_id).data(),
        C_nodes.at(constraint.node_id).data());
  }

  //   LOG(INFO) << "node_data_ size " << node_data_.size();

  auto imu_it = imu_data_.begin();
  auto node_it = node_data_.begin();
  auto prev_node_it = node_it;
  for (++node_it; node_it != node_data_.end(); ++node_it) {
    const int first_node_id = prev_node_it->first;
    const NodeSpec2D first_node_data = prev_node_it->second;
    prev_node_it = node_it;

    const int second_node_id = node_it->first;
    const NodeSpec2D second_node_data = node_it->second;

    if (second_node_id != first_node_id + 1) {
      continue;
    }

    common::Rigid3 relative_local_slam_pose;
    std::unique_ptr<common::Rigid3> relative_odometry;

    if (options_.use_local_pose) {
      // 继续构图需要特殊转换
      if (return_mapping && (first_node_id == return_mapping_node_id)) {
        relative_local_slam_pose = return_mapping_transform;
      } else {
        relative_local_slam_pose =
            common::embed3D(first_node_data.local_pose_2d.inverse() *
                            second_node_data.local_pose_2d);
      }

      // 去除异常数据
      if (relative_local_slam_pose.translation().norm() > 1.0) {
        continue;
      }

      problem.AddResidualBlock(createAutoDiffSpaCostFunction(Constraint::Pose{
                                   relative_local_slam_pose,
                                   options_.local_slam_pose_translation_weight,
                                   options_.local_slam_pose_rotation_weight}),
                               nullptr /* loss function */,
                               C_nodes.at(first_node_id).data(),
                               C_nodes.at(second_node_id).data());
    }

    if (options_.use_odom) {
      // 继续构图需要特殊转换
      if (return_mapping && (first_node_id == return_mapping_node_id)) {
        relative_odometry =
            std::make_unique<common::Rigid3>(return_mapping_transform);

      } else {
        relative_odometry =
            calculateOdometryBetweenNodes(first_node_data, second_node_data);
      }

      if (nullptr != relative_odometry) {
        // 去除异常数据
        if (relative_odometry->translation().norm() > 1.0) {
          continue;
        }

        problem.AddResidualBlock(
            createAutoDiffSpaCostFunction(Constraint::Pose{
                *relative_odometry, options_.odometry_translation_weight,
                options_.odometry_rotation_weight}),
            nullptr /* loss function */, C_nodes.at(first_node_id).data(),
            C_nodes.at(second_node_id).data());
      }
    }

    // add relative rotation constriant based on gyro integration
    if (imu_it != imu_data_.end()) {
      int segment_id = first_node_id / options_.num_node_one_segment;

      auto td = common::fromSeconds(td_data_.lidar_to_gyroscope_td[0]);
      auto first_node_time_plus = first_node_data.time + td;
      auto second_node_time_plus = second_node_data.time + td;

      if (imu_it->first > first_node_time_plus) {
        LOG(WARNING) << "The first imu stamp is greater than lidar."
                     << " imu stamp " << imu_it->first << " lidar stamp "
                     << first_node_data.time << " td " << common::toSeconds(td);
        continue;
      }

      while (std::next(imu_it) != imu_data_.end() &&
             (std::next(imu_it)->first <= first_node_time_plus)) {
        ++imu_it;
      }

      if (std::next(imu_it) == imu_data_.end()) {
        LOG(WARNING) << "Required next imu not exist.";
        continue;
      }

      bool need_preint_or_repreint = false;
      if (!node_gyr_yaw_preint_data_.count(first_node_id)) {
        need_preint_or_repreint = true;
      } else if (std::fabs(node_gyr_yaw_preint_data_.at(first_node_id)
                               ->getLinearizedTd() -
                           td_data_.lidar_to_gyroscope_td[0]) > 0.005) {
        need_preint_or_repreint = true;
        node_rotation_vel_yaw_data_.erase(first_node_id);
        node_rotation_vel_yaw_data_.erase(second_node_id);
        node_gyr_yaw_preint_data_.erase(first_node_id);
      }
      if (need_preint_or_repreint) {
        if (!node_rotation_vel_yaw_data_.count(first_node_id)) {
          double vel_yaw = InterpolateAngularVelocityYaw(
              imu_it, std::next(imu_it), first_node_time_plus);
          node_rotation_vel_yaw_data_.emplace(first_node_id, vel_yaw);
        }

        std::shared_ptr<GyroYawPreintegrator> gyr_yaw_preint(
            new GyroYawPreintegrator(
                segment_gyr_bias_data_.at(segment_id).gyro_bias[2],
                td_data_.lidar_to_gyroscope_td[0]));
        gyr_yaw_preint->computePreint(imu_data_, first_node_time_plus,
                                      second_node_time_plus, &imu_it);

        if (std::next(imu_it) == imu_data_.end()) {
          // LOG(WARNING) << "The next imu(needed by interploate) not exist.";
          continue;
        }
        if (!node_rotation_vel_yaw_data_.count(second_node_id)) {
          double vel_yaw = InterpolateAngularVelocityYaw(
              imu_it, std::next(imu_it), second_node_time_plus);
          node_rotation_vel_yaw_data_.emplace(second_node_id, vel_yaw);
        }

        node_gyr_yaw_preint_data_.emplace(first_node_id, gyr_yaw_preint);
      }

      problem.AddResidualBlock(
          new AnalyticalP3DAndB1DAndTdCostFunction2D(
              node_gyr_yaw_preint_data_.at(first_node_id).get(),
              Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity(),
              node_rotation_vel_yaw_data_.at(first_node_id),
              node_rotation_vel_yaw_data_.at(second_node_id),
              options_.gyro_yaw_rotation_weight),
          nullptr, C_nodes.at(first_node_id).data(),
          C_nodes.at(second_node_id).data(),
          C_segment_bg_z.at(segment_id).data(),
          td_data_.lidar_to_gyroscope_td.data());
    }
  }

  {
    auto segment_it = C_segment_bg_z.begin();
    auto prev_segment_it = segment_it;
    if (segment_it != C_segment_bg_z.end()) {
      segment_it++;
      for (; segment_it != C_segment_bg_z.end(); segment_it++) {
        const int first_segment_id = prev_segment_it->first;
        prev_segment_it = segment_it;
        const int second_segment_id = segment_it->first;

        CHECK(second_segment_id == first_segment_id + 1);

        problem.AddResidualBlock(
            BiasZCostFunction2D::CreateAutoDiffCostFunction(
                options_.gyro_yaw_bias_vary_weight),
            nullptr, C_segment_bg_z.at(first_segment_id).data(),
            C_segment_bg_z.at(second_segment_id).data());
      }
    }
  }

  {
    if (!C_segment_bg_z.empty()) {
      problem.AddResidualBlock(
          BiasZPriorCostFunction2D::CreateAutoDiffCostFunction(
              options_.gyro_yaw_bias_prior,
              options_.gyro_yaw_bias_prior_weight),
          nullptr, C_segment_bg_z.at(0).data());
    }
  }

  ceres::Solver::Summary summary;
  ceres::Solve(common::createCeresSolverOptions(options_.ceres_solver_options),
               &problem, &summary);
  if (options_.log_solver_summary) {
    LOG(INFO) << summary.FullReport();
  }
  // Store the result.
  for (const auto &C_submap_id_data : C_submaps) {
    submap_data_.at(C_submap_id_data.first).global_pose =
        toPose(C_submap_id_data.second);
    // LOG(INFO) << "submap id: " << C_submap_id_data.first
    //           << "  submap pose: " << C_submap_id_data.second[0] << ", "
    //           << C_submap_id_data.second[1];
  }
  for (const auto &C_node_id_data : C_nodes) {
    node_data_.at(C_node_id_data.first).global_pose_2d =
        toPose(C_node_id_data.second);
    // LOG(INFO) << "node id: " << C_node_id_data.first
    //           << "  node pose: " << C_node_id_data.second[0] << ", "
    //           << C_node_id_data.second[1];
  }

  for (const auto &C_bias_gz : C_segment_bg_z) {
    segment_gyr_bias_data_.at(C_bias_gz.first).gyro_bias[2] =
        C_bias_gz.second[0];
    std::cerr << C_bias_gz.second[0] << " |";
  }
  std::cerr << std::endl;
  std::cerr << "===== td: " << td_data_.lidar_to_gyroscope_td[0] << std::endl;
}

std::unique_ptr<common::Rigid3> OptimizationProblem::interpolateOdometry(
    const common::Time time) const {
  const auto it = odometry_data_.lower_bound(time);
  if (it == odometry_data_.end()) {
    return nullptr;
  }
  if (it == odometry_data_.begin()) {
    if (it->second.time == time) {
      return std::make_unique<common::Rigid3>(it->second.pose);
    }
    return nullptr;
  }

  const auto prev_it = std::prev(it);
  return std::make_unique<common::Rigid3>(
      common::interpolate(
          common::TimestampedTransform{prev_it->second.time,
                                       prev_it->second.pose},
          common::TimestampedTransform{it->second.time, it->second.pose}, time)
          .transform);
}

std::unique_ptr<common::Rigid3>
OptimizationProblem::calculateOdometryBetweenNodes(
    const NodeSpec2D &first_node_data,
    const NodeSpec2D &second_node_data) const {
  const std::unique_ptr<common::Rigid3> first_node_odometry =
      interpolateOdometry(first_node_data.time);
  const std::unique_ptr<common::Rigid3> second_node_odometry =
      interpolateOdometry(second_node_data.time);
  if (first_node_odometry != nullptr && second_node_odometry != nullptr) {
    common::Rigid3 relative_odometry =
        first_node_odometry->inverse() * (*second_node_odometry);
    return std::make_unique<common::Rigid3>(relative_odometry);
  }
  return nullptr;
}

void OptimizationProblem::setReturnMappingData(
    const int &node_id, const common::Rigid3 &transform) {
  return_mapping = true;
  return_mapping_node_id = node_id;
  return_mapping_transform = transform;
}

}  // namespace backend
}  // namespace slam2d_core
