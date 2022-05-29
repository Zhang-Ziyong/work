#ifndef _SLAM_SYSTEM_H_
#define _SLAM_SYSTEM_H_

#include <map>
#include <string>

#include "localization.hpp"
#include "mapping.hpp"
#include "state_machine/slam_state_machine.hpp"

namespace slam2d_core {
namespace slam_system {

class SlamSystem {
 public:
  ~SlamSystem();
  static SlamSystem *getInstance();

  // mapping api
  void startMapping(const std::string &realtime_map_file);
  void canceMapping();
  void mappingAddOdom(const common::OdometryData &odom_data);
  void mappingAddScan(const common::TimedPointCloudData &time_point_cloud_data);
  void mappingAddImu(const common::ImuData &imu_data);
  bool getMappingLocalizationResult(const common::Time &time,
                                    common::Rigid3 &pose);
  frontend::PaintSubmapSlicesResult paintSubmapSlices();
  frontend::PaintSubmapSlicesResult paintSubmapSlices(double resolution);
  void setParams(
      const slam2d_core::backend::PoseGraphOptions &pose_graph_options,
      const slam2d_core::frontend::LocalTrajectoryBuilderOptions
          &local_trajectory_options);

  void setParams(
      const slam2d_core::slam_system::LocalizationOptions &loc_options,
      const slam2d_core::amcl::AmclOptions &amcl_options,
      const slam2d_core::msf::MultiSensorsFusionOptions &msf_options,
      const slam2d_core::scan_matcher::GlobalScanMatcher2DOptions &gsm_options);

  inline common::Rigid3 getCurrentPose() {
    std::lock_guard<std::mutex> guard(pose_mutex_);
    return current_pose_;
  }
  inline bool getLocStatus() { return loc_status_; }

  bool getLastTrajectoryNode(int &id, common::Rigid3 &node_pose);

  bool getTrajectoryNodePose(const int &id, common::Rigid3 &node_pose);

  std::map<int, common::TrajectoryNodePose> getTrajectoryNodePoses();

  std::map<int, backend::SubmapPose> getAllSubmapPoses() const;

  std::vector<backend::Constraint> constraints() const;

  void setReturnMappingPose(const common::Rigid3 &global_pose);

  bool requestSaveMap(const std::string &file_name);

  void runFinalOptimization();

  // localization api
  void startLocalization(const std::string &map_file_path,
                         const InitModle &init_modle,
                         const slam2d_core::common::Rigid3 &init_pose);
  void stopLocalization();

  void localizationAddOdomData(
      const slam2d_core::common::OdometryData &odom_data);
  void localizationAddScanData(
      const slam2d_core::amcl::LaserScanData &scan_data);

  inline bool isLocalizationInit() {
    return localization_.isLocalizationInit();
  }

  void getLocalizationResult(Eigen::Vector3d &result_cov,
                             slam2d_core::common::Rigid3 &result_pose,
                             bool &loc_status);

  void getCurrentPFSet(std::vector<slam2d_core::amcl::Sample> &current_set,
                       bool &pf_set_pub_flag) {
    return localization_.getCurrentPFSet(current_set, pf_set_pub_flag);
  }

  inline std::shared_ptr<slam2d_core::amcl::OccupancyGrid> getLocMap() {
    return localization_.getMap();
  }
  bool getJudgeSaveMap() { return judge_save_map_; }

  std::mutex map_mutex_;
  void mapLock() { map_mutex_.lock(); }
  void mapUnLock() { map_mutex_.unlock(); }

  inline void setLaserTf(const common::Rigid3 &laser_tf) {
    laser_tf_ = laser_tf;
  }

  inline common::Rigid3 getLaserTf() { return laser_tf_; }

  bool serializeStateToFile(const std::string &filename);

  void loadSerializeStateFromFile(const std::string &state_filename);

  inline bool getLaserOdometryData(
      slam2d_core::common::OdometryData &OdometryData) {
    return localization_.getLaserOdometryData(OdometryData);
  }

  inline slam2d_core::state_machine::SENSOR_STATUS getSensorStatus() {
    return localization_.getCurrentSensorStatus();
  }

  inline void setSensorStatusNormal() {
    return localization_.setSensorStatusNormal();
  }

  // 设置电梯参数
  inline void setElevatorStatus(bool is_elevator_params) {
    localization_.setElevatorStatus(is_elevator_params);
  }

  // inline void getOdometryDataUpdate(
  //     slam2d_core::common::OdometryData &odometry_data) {
  //   localization_.getOdometryDataUpdate(odometry_data);
  // }

  //
  inline void getLaserOdomCloud(
      pcl::PointCloud<PointType>::Ptr &pre_aligned_laser_cloud,
      pcl::PointCloud<PointType>::Ptr &local_laser_cloud_map) {
    localization_.getLaserOdomCloud(pre_aligned_laser_cloud,
                                    local_laser_cloud_map);

    if (pre_aligned_laser_cloud == nullptr) {
      LOG(ERROR) << "pre_aligned_laser_cloud == nullptr";
    }
    if (local_laser_cloud_map == nullptr) {
      LOG(ERROR) << "local_laser_cloud_map == nullptr";
    }
  }

  inline void getFirstMapToLaserTransfrom(
      slam2d_core::common::Rigid3 &first_transfrom) {
    localization_.getFirstMapToLaserTransfrom(first_transfrom);
  }

 private:
  SlamSystem();

  common::Rigid3 current_pose_;
  bool loc_status_;
  common::Rigid3 laser_tf_;
  std::mutex pose_mutex_;
  Mapping mapping_;
  Localization localization_;
  static SlamSystem *ptr_system_;
  std::shared_ptr<state_machine::SlamStateMachine> ptr_state_machine_;
  bool judge_save_map_ = false;
};

typedef std::shared_ptr<SlamSystem> SlamSystemPtr;

}  // namespace slam_system
}  // namespace slam2d_core

#endif