/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 201９, CVTE.
 * All rights reserved.
 *
 *@file keyframe.hpp
 *
 *@brief
 * 1.关键帧类
 *
 *@author Wei Zhang(zhangwei@cvte.com)
 *@modify Yun Su(robosu12@gmail.com)
 *@version 0.5
 *@data 2022-03-16
 ************************************************************************/
#ifndef KRYFRAME_HPP_
#define KRYFRAME_HPP_

#include <memory>
#include <mutex>
#include <atomic>

#include "common/data_struct/pc_base.hpp"
#include "common/math_base/slam_math.hpp"
#include "common/data_struct/sensors_type.hpp"
namespace cvte_lidar_slam {
class KeyFrame {
 public:
  KeyFrame() {}

  /**
   *KeyFrame
   *@brief
   *关键帧构造类
   *
   *@param[in] pos-关键帧位置
   *@param[in] cornerCloud-线点点云
   *@param[in] surfCloud-面点点云
   **/
  KeyFrame(const FramePosition &pos, const laserCloud::Ptr cornerCloud,
           const laserCloud::Ptr surfCloud);

  /**
   *KeyFrame
   *@brief
   *关键帧构造类
   *
   *@param[in] frame_info-关键帧信息
   *@param[in] cornerCloud-线点点云
   *@param[in] surfCloud-面点点云
   **/
  KeyFrame(const KeyFrameInfo &frame_info, const laserCloud::Ptr cornerCloud,
           const laserCloud::Ptr surfCloud);

  /**
   *KeyFrame
   *@brief
   *关键帧构造类
   *
   *@param[in] ID-关键帧索引
   *@param[in] time_stamp-关键帧时间戳
   *@param[in] T_wo-关键帧里程计pose
   *@param[in] T_wb-关键帧pose
   *@param[in] cornerCloud-线点点云
   *@param[in] surfCloud-面点点云
   **/
  KeyFrame(const size_t ID, const double time_stamp, const Mat34d &T_wo,
           const Mat34d &T_wb, const laserCloud::Ptr cornerCloud,
           const laserCloud::Ptr surfCloud);

  /**
   *KeyFrame
   *@brief
   *关键帧构造类
   *
   *@param[in] ID-关键帧索引
   *@param[in] time_stamp-关键帧时间戳
   *@param[in] T_wo-关键帧里程计pose
   *@param[in] T_wb-关键帧pose
   *@param[in] cornerCloud-线点点云
   *@param[in] surfCloud-面点点云
   *@param[in] withoutdGroundCloud-没有地面点的点云(下采样)
   **/
  KeyFrame(const size_t ID, const double time_stamp, const Mat34d &T_wo,
           const Mat34d &T_wb, const laserCloud::Ptr cornerCloud,
           const laserCloud::Ptr surfCloud,
           const laserCloud::Ptr withoutdGroundCloud);

  /**
   *KeyFrame
   *@brief
   *关键帧构造类
   *
   *@param[in] ID-关键帧索引
   *@param[in] time_stamp-关键帧时间戳
   *@param[in] T_wo-关键帧里程计pose
   *@param[in] T_wb-关键帧pose
   *@param[in] linear-关键帧linear
   *@param[in] angular-关键帧angular
   *@param[in] cornerCloud-线点点云
   *@param[in] surfCloud-面点点云
   *@param[in] withoutdGroundCloud-没有地面点的点云(下采样)
   **/
  KeyFrame(const size_t ID, const double time_stamp, const Mat34d &T_wo,
           const Mat34d &T_wb, const double linear, const double angular,
           const laserCloud::Ptr cornerCloud, const laserCloud::Ptr surfCloud,
           const laserCloud::Ptr withoutdGroundCloud,
           const int environment_flag);

  /**
   *KeyFrame
   *@brief
   *关键帧构造类
   *
   *@param[in] ID-关键帧索引
   *@param[in] time_stamp-关键帧时间戳
   *@param[in] T_wo-关键帧里程计pose
   *@param[in] T_wb-关键帧pose
   *@param[in] cornerCloud-线点点云
   *@param[in] surfCloud-面点点云
   *@param[in] withoutdGroundCloud-没有地面点的点云(下采样)
   *@param[in] IMU-IMU measure(contain posture)
   **/
  KeyFrame(const size_t ID, const double time_stamp, const Mat34d &T_wo,
           const Mat34d &T_wb, const laserCloud::Ptr cornerCloud,
           const laserCloud::Ptr surfCloud, const laserCloud::Ptr rawCloud,
           const laserCloud::Ptr withoutdGroundCloud, const ImuMeasure IMU);

  /**
   *KeyFrame
   *@brief
   *关键帧构造类
   *
   *@param[in] ID-关键帧索引
   *@param[in] time_stamp-关键帧时间戳
   *@param[in] T_wo-关键帧里程计pose
   *@param[in] T_wb-关键帧pose
   *@param[in] cornerCloud-线点点云
   *@param[in] surfCloud-面点点云
   *@param[in] withoutdGroundCloud-没有地面点的点云(下采样)
   *@param[in] gps_pos-gps位置
   *@param[in] gps_cov-gps位置协防差
   **/
  KeyFrame(const size_t ID, const double time_stamp, const Mat34d &T_wo,
           const Mat34d &T_wb, const laserCloud::Ptr cornerCloud,
           const laserCloud::Ptr surfCloud,
           const laserCloud::Ptr withoutdGroundCloud, const Vec3d &gps_pos,
           const double gps_cov);

  /**
   *KeyFrame
   *@brief
   *关键帧构造类
   *
   *@param[in] ID-关键帧索引
   *@param[in] time_stamp-关键帧时间戳
   *@param[in] T_wo-关键帧里程计pose
   *@param[in] T_wb-关键帧pose
   *@param[in] cornerCloud-线点点云
   *@param[in] surfCloud-面点点云
   *@param[in] gps_pos-gps位置
   *@param[in] gps_cov-gps位置协防差
   **/
  KeyFrame(const size_t ID, const double time_stamp, const Mat34d &T_wo,
           const Mat34d &T_wb, const laserCloud::Ptr cornerCloud,
           const laserCloud::Ptr surfCloud, const Vec3d &gps_pos,
           const double gps_cov);

  KeyFrame(const size_t ID, const double time_stamp, const Mat34d &T_wo,
           const Mat34d &T_wb, const double linear, const double angular,
           const laserCloud::Ptr cornerCloud, const laserCloud::Ptr surfCloud,
           const laserCloud::Ptr withoutdGroundCloud,
           const int environment_flag, const Vec6d &h_matrix);

  ~KeyFrame();

  /**
   *updatePose
   *@brief
   *更新关键帧pose
   *
   *@param[in] pose-关键帧pose
   **/
  void updatePose(const Mat34d &pose) {
    std::unique_lock<std::mutex> lock(mutex_pose_);
    T_wb_ = pose;
    pos_.x = pose(0, 3);
    pos_.y = pose(1, 3);
    pos_.z = pose(2, 3);
  }

  /**
   *updatePose
   *@brief
   *更新关键帧pose
   *
   *@param[in] pose-关键帧pose
   **/
  void updateLaserOdomPose(const Mat34d &pose) { T_wl_ = pose; }

  inline Mat34d getLaserOdomPose() { return T_wl_; }

  /**
   *getWheelPose
   *@brief
   *获取关键帧里程计pose
   *
   *@return Mat34d
   **/
  inline Mat34d getWheelPose() { return T_wo_; }

  void updateWheelPose(const Mat34d &pose) { T_wo_ = pose; }

  /**
   *getMapMatchPose
   *@brief
   *获取关键帧map match pose
   *
   *@return Mat34d
   **/
  inline Mat34d getMapMatchPose() { return T_wm_; }

  /**
   *getPose
   *@brief
   *获取关键帧pose
   *
   *@return Mat34d
   **/
  inline Mat34d getPose() {
    std::unique_lock<std::mutex> lock(mutex_pose_);
    return T_wb_;
  }

  /**
   *getPosition
   *@brief
   *获取关键帧位置
   *
   *@return FramePosition
   **/
  inline FramePosition getPosition() {
    std::unique_lock<std::mutex> lock(mutex_pose_);
    return pos_;
  }

  /**
   *getGPSStatus
   *@brief
   *获取关键帧gps状态
   *
   *@return false-无gps，true-有gps
   **/
  inline bool getGPSStatus() { return has_gps_; }

  /**
   *isActived
   *@brief
   *获取关键帧状态
   *
   *@return false-点云无效，true-点云有效
   **/
  bool isActived() const { return is_actived_; }

  /**
   *setActivedSatus
   *@brief
   *设置关键帧状态
   *@param[in] false-点云无效，true-点云有效
   **/
  void setActivedSatus(const bool is_actived) { is_actived_ = is_actived; }

 public:
  size_t index_;       ///< 点云索引ID
  double time_stamp_;  ///< 点云时间戳
  Mat34d T_wo_;        ///< wheel odom
  Mat34d T_wb_;        ///< body pose
  Mat34d T_wl_;        ///< laser odom
  Mat34d T_wm_;        ///< map match pose
  double linear;
  double angular;
  double moved_distance_ = 0.0;
  const Vec3d gps_pos_;                   ///< gps position
  double gps_cov_;                        ///< gps covariance>
  Mat6d laser_odom_cov_;                  ///< 雷达里程计协防差
  Mat6d wheel_odom_cov_;                  ///< 轮式里程计协防差
  Mat6d map_cov_;                         ///< 地图观测协防差
  laserCloud::Ptr corner_cloud_;          ///< 稠密线点点云
  laserCloud::Ptr surf_cloud_;            ///< 稠密面点点云
  laserCloud::Ptr raw_cloud_;             ///< 稠密面点点云
  laserCloud::Ptr without_ground_cloud_;  ///< 无地面点的点云，下采样
  std::shared_ptr<KeyFrame> ptr_last_keyframe_ = nullptr;  ///<上一帧指针
  ImuMeasure IMU_;

  bool has_gps_;                   ///< gps标志位
  bool has_imu_ = false;           ///< gps标志位
  bool is_actived_ = true;         ///< 激活状态位
  bool is_wheel_skidded_ = false;  ///< 打滑标志位

  int environment_flag_;
  Vec6d H_matrix_;  //< 点云H矩阵，用于判断退化方向

 private:
  FramePosition pos_;      ///< 数据帧位置
  std::mutex mutex_pose_;  ///<对象锁>

};  // end of class

typedef std::shared_ptr<KeyFrame> KeyFramePtr;

}  // namespace cvte_lidar_slam

#endif  // KRYFRAME_HPP_