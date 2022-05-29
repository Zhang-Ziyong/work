#include "slam_system/slam_system.hpp"

namespace slam2d_core {
namespace slam_system {

SlamSystem *SlamSystem::ptr_system_ = nullptr;

SlamSystem::SlamSystem() {
  ptr_state_machine_ = state_machine::SlamStateMachine::getInstance();
}

SlamSystem::~SlamSystem() {
  //   run_opt_ = false;
  //   shutdown();
}

SlamSystem *SlamSystem::getInstance() {
  static SlamSystem lidar_slam_system;
  return &lidar_slam_system;
}

void SlamSystem::setParams(
    const slam2d_core::backend::PoseGraphOptions &pose_graph_options,
    const slam2d_core::frontend::LocalTrajectoryBuilderOptions
        &local_trajectory_options) {
  mapping_.setParams(pose_graph_options, local_trajectory_options);
}

void SlamSystem::setParams(
    const slam2d_core::slam_system::LocalizationOptions &loc_options,
    const slam2d_core::amcl::AmclOptions &amcl_options,
    const slam2d_core::msf::MultiSensorsFusionOptions &msf_options,
    const slam2d_core::scan_matcher::GlobalScanMatcher2DOptions &gsm_options) {
  localization_.setParams(loc_options, amcl_options, msf_options, gsm_options);
}

void SlamSystem::startMapping(const std::string &realtime_map_file) {
  mapping_.startMapping(realtime_map_file);
}

void SlamSystem::canceMapping() {
  mapping_.canceMapping();
  judge_save_map_ = false;
}

void SlamSystem::mappingAddOdom(const common::OdometryData &odom_data) {
  mapping_.addOdomData(odom_data);
}

void SlamSystem::mappingAddImu(const common::ImuData &imu_data) {
  mapping_.addImuData(imu_data);
}

void SlamSystem::mappingAddScan(
    const common::TimedPointCloudData &time_point_cloud_data) {
  mapping_.addScanData(time_point_cloud_data);
}

void SlamSystem::runFinalOptimization() {
  mapping_.runFinalOptimization();
}

bool SlamSystem::getMappingLocalizationResult(const common::Time &time,
                                              common::Rigid3 &pose) {
  if (!mapping_.getLocalizationResult(time, pose)) {
    pose = current_pose_;
    return false;
  }
  std::lock_guard<std::mutex> guard(pose_mutex_);
  current_pose_ = pose;
  return true;
}

frontend::PaintSubmapSlicesResult SlamSystem::paintSubmapSlices() {
  return mapping_.paintSubmapSlices();
}

frontend::PaintSubmapSlicesResult SlamSystem::paintSubmapSlices(
    double resolution) {
  return mapping_.paintSubmapSlices(resolution);
}

bool SlamSystem::getLastTrajectoryNode(int &id, common::Rigid3 &node_pose) {
  return mapping_.getLastTrajectoryNode(id, node_pose);
}

bool SlamSystem::getTrajectoryNodePose(const int &id,
                                       common::Rigid3 &node_pose) {
  return mapping_.getTrajectoryNodePose(id, node_pose);
}

std::map<int, common::TrajectoryNodePose> SlamSystem::getTrajectoryNodePoses() {
  return mapping_.getTrajectoryNodePoses();
}

std::map<int, backend::SubmapPose> SlamSystem::getAllSubmapPoses() const {
  return mapping_.getAllSubmapPoses();
}

std::vector<backend::Constraint> SlamSystem::constraints() const {
  return mapping_.constraints();
}

void SlamSystem::setReturnMappingPose(const common::Rigid3 &global_pose) {
  mapping_.setReturnMappingPose(global_pose);
}

bool SlamSystem::requestSaveMap(const std::string &file_name) {
  judge_save_map_ = true;
  LOG(INFO) << " cant pub map ";
  return mapping_.requestSaveMap(file_name);
}

bool SlamSystem::serializeStateToFile(const std::string &filename) {
  return mapping_.serializeStateToFile(true, filename);
}

void SlamSystem::loadSerializeStateFromFile(const std::string &state_filename) {
  mapping_.loadSerializeStateFromFile(state_filename);
}

void SlamSystem::startLocalization(
    const std::string &map_file_path, const InitModle &init_modle,
    const slam2d_core::common::Rigid3 &init_pose) {
  loc_status_ = false;
  localization_.startLocalization(map_file_path, init_modle, init_pose);
}

void SlamSystem::localizationAddOdomData(
    const slam2d_core::common::OdometryData &odom_data) {
  localization_.addOdomData(odom_data);
}

void SlamSystem::localizationAddScanData(
    const slam2d_core::amcl::LaserScanData &scan_data) {
  localization_.addScanData(scan_data);
}

void SlamSystem::getLocalizationResult(Eigen::Vector3d &result_cov,
                                       slam2d_core::common::Rigid3 &result_pose,
                                       bool &loc_status) {
  localization_.getLocalizationResult(result_cov, result_pose, loc_status);

  std::lock_guard<std::mutex> guard(pose_mutex_);
  current_pose_ = result_pose;
  loc_status_ = loc_status;
}

void SlamSystem::stopLocalization() {
  localization_.stopLocalization();
}

}  // namespace slam_system
}  // namespace slam2d_core