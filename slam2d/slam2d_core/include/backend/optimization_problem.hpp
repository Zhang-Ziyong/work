#ifndef _OPTIMIZATION_PROBLEM_H_
#define _OPTIMIZATION_PROBLEM_H_

#include <array>
#include <map>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "backend/pose_graph_data.hpp"
#include "ceres/ceres.h"
#include "common/ceres_solver_options.hpp"
#include "common/odometry_data.hpp"
#include "common/imu_data.hpp"
#include "common/rigid_transform.hpp"
#include "glog/logging.h"

namespace slam2d_core {
namespace backend {
class GyroYawPreintegrator;

class OptimizationProblemOptions {
 public:
  double huber_scale = 1e1;
  double acceleration_weight = 1.1e2;
  double rotation_weight = 1.6e4;
  double local_slam_pose_translation_weight = 1e5;
  double local_slam_pose_rotation_weight = 1e5;
  double odometry_translation_weight = 1e5;
  double odometry_rotation_weight = 1e5;
  double gyro_yaw_rotation_weight = 1e6;
  double gyro_yaw_bias_vary_weight = 1e7;
  double gyro_yaw_bias_prior = 0.;
  double gyro_yaw_bias_prior_weight = 1e7;
  int num_node_one_segment = 300;

  bool use_odom = true;
  bool use_local_pose = true;
  bool log_solver_summary = false;

  common::CeresSolverOptions ceres_solver_options;
};

class OptimizationProblem {
 public:
  explicit OptimizationProblem(const OptimizationProblemOptions &options);
  ~OptimizationProblem();

  OptimizationProblem(const OptimizationProblem &) = delete;
  OptimizationProblem &operator=(const OptimizationProblem &) = delete;

  void addImuData(const common::ImuData &imu_data);

  void addOdometryData(const common::OdometryData &odometry_data);

  void addTrajectoryNode(const NodeSpec2D &node_data);

  void insertTrajectoryNode(const int &node_id, const NodeSpec2D &node_data);

  void trimTrajectoryNode(const int &node_id);

  void addSubmap(const common::Rigid2 &global_submap_pose);

  void insertSubmap(const int &submap_id,
                    const common::Rigid2 &global_submap_pose);

  void trimSubmap(const int &submap_id);
  void setMaxNumIterations(int32_t max_num_iterations);

  void solve(const std::vector<Constraint> &constraints);

  const std::map<int, NodeSpec2D> &node_data() const { return node_data_; }
  const std::map<int, SubmapSpec2D> &submap_data() const {
    return submap_data_;
  }
  const std::map<common::Time, common::OdometryData> &odometry_data() const {
    return odometry_data_;
  }

  void setReturnMappingData(const int &node_id,
                            const common::Rigid3 &transform);

 private:
  struct TrajectoryData {
    int node_id = 0;
    int submap_id = 0;
  };

  struct GyroBiasData {
    std::array<double, 3> gyro_bias{{0., 0., 0.}};
    // std::array<double, 3> accel_bias{{0., 0., 0.}};
  };

  struct SensorsTimeOffset {
    std::array<double, 1> lidar_to_gyroscope_td{
        {0.}};  // t_gyroscope = t_lidar + offset[s]
    std::array<double, 1> lidar_to_odometry_td{
        {0.}};  // t_odometry = t_lidar + offset[s]
  };

  void trimOdomtryData(const int &node_id);

  std::unique_ptr<common::Rigid3> interpolateOdometry(common::Time time) const;

  std::unique_ptr<common::Rigid3> calculateOdometryBetweenNodes(
      const NodeSpec2D &first_node_data,
      const NodeSpec2D &second_node_data) const;

  TrajectoryData trajectory_data_;
  OptimizationProblemOptions options_;
  std::map<int, NodeSpec2D> node_data_;
  std::map<int, SubmapSpec2D> submap_data_;
  std::map<common::Time, common::OdometryData> odometry_data_;

  std::map<common::Time, common::ImuData> imu_data_;
  std::map<int, GyroBiasData> segment_gyr_bias_data_;
  std::map<int, std::shared_ptr<GyroYawPreintegrator>>
      node_gyr_yaw_preint_data_;
  std::map<int, double> node_rotation_vel_yaw_data_;
  SensorsTimeOffset td_data_;

  bool return_mapping = false;
  int return_mapping_node_id = 0;
  common::Rigid3 return_mapping_transform;
};

}  // namespace backend
}  // namespace slam2d_core

#endif
