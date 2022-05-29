/*
 * @Author: your name
 * @Date: 2020-10-31 11:46:30
 * @LastEditTime: 2021-07-02 10:47:32
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath:
 * /src/tracking_motion/tracking_motion_adapter/tracking_motion_adapter.hpp
 */
#ifndef TRACKING_MOTION_ADAPTER_HPP_
#define TRACKING_MOTION_ADAPTER_HPP_
#include "tracking_motion_adapter_ros2.hpp"
#include "tracking_motion_config.hpp"
#include "kalman_filter.hpp"
#include "track_base.hpp"
#include "laser_object_track.hpp"
#include "potential_controller.hpp"
#include "controller_base.hpp"
#include <mutex>
#include <chrono>
#include <map>
namespace TRACKING_MOTION {

enum STATUS {
  UNKNOW,
  TRACKING,
};

enum TRACKINGSTATUS {
  IDLE,
  INIT,
  SUCCEED,
  LOST,
};

class TrackingMotionAdapter {
 public:
  TrackingMotionAdapter();
  TrackingMotionAdapter(const TrackingMotionAdapter &obj) = delete;
  const TrackingMotionAdapter &operator=(const TrackingMotionAdapter &obj) =
      delete;
  void start();
  void stop();
  void spin();

 private:
  std::shared_ptr<TrackingAdapterRos2> ptr_tracking_adapter_ros2_;
  std::shared_ptr<LaserObjectTrack> ptr_track_base_;
  std::shared_ptr<PotentialController> ptr_controller_;
  std::shared_ptr<KalmanFilter> ptr_kalman_tracker_;

  std::chrono::time_point<std::chrono::system_clock> last_odom_time_;
  std::chrono::time_point<std::chrono::system_clock> last_scan_time_;
  std::chrono::time_point<std::chrono::system_clock> last_cloud_time_;
  std::chrono::time_point<std::chrono::system_clock> last_target_time_;

  std::mutex odom_pose_mutex_;
  std::mutex odom_time_mutex_;
  std::mutex scan_time_mutex_;
  std::mutex cloud_time_mutex_;
  std::mutex target_time_mutex_;

  bool first_odom_;
  // bool tracking_lost_;

  STATUS state_;
  TRACKINGSTATUS tracking_state_;

  std::map<TRACKINGSTATUS, std::string> map_status_str_;

  Mat34d T_lo_;
  Mat34d odom_pose_;
  Mat34d first_odom_pose_;
  Mat34d target_in_world_;
  Mat34d target_pose_;

  CmdVel cur_rev_vel_;

  TrackingMotionConfig config_;

  double base_link_to_laser_;

  void odomCallback(const Mat34d &pose, const CmdVel &rev_vel);
  void scanCallback(const std::vector<Eigen::Vector2d> &scan);
  void pointCloudCallback(const laserCloud::Ptr ptr_point_cloud);
  void missionManagerCallback(std::string &, std::string &);
  void targetCallback(const Mat34d &pose);

  void updateScanTime();
  void updateOdomTime();
  void updatePointCloudTime();
  void updateTargetTime();

  std::chrono::duration<double> calcuOdomTimeDiff();
  std::chrono::duration<double> calcuScanTimeDiff();
  std::chrono::duration<double> calcuPointCloudTimeDiff();
  std::chrono::duration<double> calcuTargetTimeDiff();
};

}  // namespace TRACKING_MOTION

#endif