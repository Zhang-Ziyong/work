#ifndef _MAPPING_H_
#define _MAPPING_H_

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "backend/global_trajectory_builder.hpp"
#include "backend/pose_graph_data.hpp"
#include "common/fixed_ratio_sampler.hpp"
#include "frontend/local_trajectory_builder.hpp"
#include "frontend/submap_painter.hpp"
#include "common/probability_grid.hpp"
#include "common/probability_values.hpp"

namespace slam2d_core {
namespace slam_system {

using namespace slam2d_core::common;

class Mapping {
 public:
  Mapping();
  ~Mapping();

  Mapping(const Mapping &) = delete;
  Mapping &operator=(const Mapping &) = delete;

  bool startMapping(const std::string &realtime_map_file);
  bool canceMapping();
  void runFinalOptimization();
  void addOdomData(const common::OdometryData &odom_data);
  void addScanData(const common::TimedPointCloudData &time_point_cloud_data);
  void addImuData(const common::ImuData &imu_data);
  bool getLocalizationResult(const common::Time &time, common::Rigid3 &pose);
  frontend::PaintSubmapSlicesResult paintSubmapSlices();
  frontend::PaintSubmapSlicesResult paintSubmapSlices(double resolution);

  void setParams(
      const slam2d_core::backend::PoseGraphOptions &pose_graph_options,
      const slam2d_core::frontend::LocalTrajectoryBuilderOptions
          &local_trajectory_options);
  bool requestSaveMap(const std::string &file_name);

  bool hasTrajectoryNode();
  bool getLastTrajectoryNode(int &id, common::Rigid3 &node_pose);
  bool getTrajectoryNodePose(const int &id, common::Rigid3 &node_pose);

  std::map<int, common::TrajectoryNodePose> getTrajectoryNodePoses();

  std::map<int, backend::SubmapPose> getAllSubmapPoses() const;

  std::vector<backend::Constraint> constraints() const;

  void setReturnMappingPose(const common::Rigid3 &global_pose);

  bool serializeStateToFile(bool include_unfinished_submaps,
                            const std::string &filename);

  void loadSerializeStateFromFile(const std::string &state_filename);

 private:
  void drawMapCallback();
  void drawMap();
  void saveOccupancyGridToMapfile();
  bool saveMapToFile(
      const slam2d_core::frontend::PaintSubmapSlicesResult &painted_slices,
      const double &resolution, const std::string &file_name);

  bool saveMapYaml(
      const slam2d_core::frontend::PaintSubmapSlicesResult &painted_slices,
      const double &resolution, const std::string &file_name);

  std::string realtime_map_file_ = "/tmp/map";

  bool stop_thread_ = true;
  std::thread draw_map_thread_;
  double scan_sampling_ratio_ = 0.3;      ///< scan数据采样率
  double odometry_sampling_ratio_ = 0.3;  ///< 里程数据采样率
  double imu_sampling_ratio_ = 1.0;       ///< IMU数据采样率

  slam2d_core::backend::PoseGraphOptions pose_graph_options_;
  slam2d_core::frontend::LocalTrajectoryBuilderOptions
      local_trajectory_options_;

  std::unique_ptr<FixedRatioSampler> scan_sampler_ptr_ =
      nullptr;  ///< scan数据采样器
  std::unique_ptr<FixedRatioSampler> odometry_sampler_ptr_ =
      nullptr;  ///<里程数据采样器
  std::unique_ptr<FixedRatioSampler> imu_sampler_ptr_ =
      nullptr;  /// IMU数据采样器
  std::unique_ptr<slam2d_core::backend::GlobalTrajectoryBuilder>
      global_trajectory_builder_ptr_ =
          nullptr;  ///< 全局轨迹生成器，slam2d_core的主要对外接口类
  std::map<int, slam2d_core::frontend::SubmapSlice> submap_slices_;

  std::mutex submap_slices_mutex_;
  std::mutex get_node_mutex_;
};

}  // namespace slam_system
}  // namespace slam2d_core

#endif