/*
 * @Author: your name
 * @Date: 2020-10-31 10:21:01
 * @LastEditTime: 2020-12-08 19:20:37
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /src/tracking_motion/laser_object_track/laser_object_track.hpp
 */
#ifndef LASER_OBJECT_TRACK_HPP_
#define LASER_OBJECT_TRACK_HPP_
#include "pc_base.hpp"
#include "track_base.hpp"
#include "tracking_motion_config.hpp"
#include <pcl/filters/extract_indices.h>
#include <mutex>

namespace TRACKING_MOTION {
class LaserObjectTrack final : public TrackBase {
 public:
  LaserObjectTrack(const LaserObjectTrackConfig &config);
  ~LaserObjectTrack() {}
  LaserObjectTrack(const LaserObjectTrack &obj) = delete;
  const LaserObjectTrack &operator=(const LaserObjectTrack &obj) = delete;
  virtual void setPredictPose(const Mat34d &predict_pose) override;
  virtual bool updateTracking(const laserCloud::Ptr laser_cloud_in) override;
  virtual void getTrackingPose(Mat34d &tracking_pose) override;
  virtual bool initTracking(const laserCloud::Ptr laser_cloud_in) override;
  virtual bool isInitTrack() override;

 private:
  bool gridMapFilter(const laserCloud::Ptr ptr_cloud_in,
                     const laserCloud::Ptr ptr_no_ground,
                     const laserCloud::Ptr ptr_ground);

  bool is_init_track_;

  std::vector<std::vector<PointType> *> vvp_grid_filled_;
  std::vector<std::vector<std::vector<PointType>>> grid_pts_;

  Mat34d predict_pose_;
  Mat34d tracking_pose_;

  std::mutex predict_pose_mutex_;
  std::mutex tracking_pose_mutex_;

  LaserObjectTrackConfig config_;
};
}  // namespace TRACKING_MOTION
#endif