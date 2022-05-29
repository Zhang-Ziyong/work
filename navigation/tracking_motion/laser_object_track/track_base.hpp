/*
 * @Author: your name
 * @Date: 2020-10-29 16:34:49
 * @LastEditTime: 2020-11-14 11:08:47
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /src/tracking_motion/laser_object_track/track_base.hpp
 */
#ifndef TRACK_BASE_HPP_
#define TRACK_BASE_HPP_
#include "pc_base.hpp"
#include "slam_math.hpp"
#include "tracking_motion_config.hpp"
namespace TRACKING_MOTION {
class TrackBase {
 public:
  virtual ~TrackBase() {}
  TrackBase() {}
  virtual void setPredictPose(const Mat34d &predict_pose) = 0;
  virtual bool updateTracking(const laserCloud::Ptr laser_cloud_in) = 0;
  virtual void getTrackingPose(Mat34d &tracking_pose) = 0;
  virtual bool initTracking(const laserCloud::Ptr laser_cloud_in) = 0;
  virtual bool isInitTrack() = 0;

 private:
};
}  // namespace TRACKING_MOTION
#endif