/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 201９, CVTE.
 * All rights reserved.
 *
 *@file keyframe.cpp
 *
 *@brief
 * 1.关键帧类
 *
 *@author Wei Zhang(zhangwei@cvte.com)
 *@version 1.0
 *@data 2019-11-04
 ************************************************************************/
#include "common/data_struct/keyframe.hpp"
#include "common/math_base/slam_math.hpp"

namespace cvte_lidar_slam {

KeyFrame::KeyFrame(const FramePosition &pos, const laserCloud::Ptr cornerCloud,
                   const laserCloud::Ptr surfCloud)
    : has_gps_(false) {
  pos_ = pos;
  index_ = (size_t) pos.intensity;
  corner_cloud_.reset(new laserCloud());
  surf_cloud_.reset(new laserCloud());
  *corner_cloud_ = *cornerCloud;
  *surf_cloud_ = *surfCloud;
}

KeyFrame::KeyFrame(const KeyFrameInfo &frame_info,
                   const laserCloud::Ptr cornerCloud,
                   const laserCloud::Ptr surfCloud)
    : index_(frame_info.frame_id),
      time_stamp_(0.),
      T_wo_(frame_info.pose),
      T_wb_(frame_info.pose),
      gps_pos_(frame_info.gps_pos),
      gps_cov_(frame_info.gps_cov),
      has_gps_(frame_info.has_gps) {
  laser_odom_cov_ = Mat6d::Identity();
  const double &distance_error = frame_info.pose_cov;
  laser_odom_cov_.block<3, 3>(0, 0) = 0.4 * distance_error * Mat3d::Identity();
  laser_odom_cov_.block<3, 3>(3, 3) = distance_error * Mat3d::Identity();
  map_cov_ = laser_odom_cov_;
  pos_.x = frame_info.pose(0, 3);
  pos_.y = frame_info.pose(1, 3);
  pos_.z = frame_info.pose(2, 3);
  corner_cloud_.reset(new laserCloud());
  surf_cloud_.reset(new laserCloud());
  *corner_cloud_ = *cornerCloud;
  *surf_cloud_ = *surfCloud;
}

KeyFrame::KeyFrame(const size_t ID, const double time_stamp, const Mat34d &T_wo,
                   const Mat34d &T_wb, const laserCloud::Ptr cornerCloud,
                   const laserCloud::Ptr surfCloud)
    : index_(ID),
      time_stamp_(time_stamp),
      T_wo_(T_wo),
      T_wb_(T_wb),
      has_gps_(false) {
  pos_.x = T_wb(0, 3);
  pos_.y = T_wb(1, 3);
  pos_.z = T_wb(2, 3);
  laser_odom_cov_ = 0.3 * Mat6d::Identity();
  map_cov_ = laser_odom_cov_;
  corner_cloud_.reset(new laserCloud());
  surf_cloud_.reset(new laserCloud());
  *corner_cloud_ = *cornerCloud;
  *surf_cloud_ = *surfCloud;
}

KeyFrame::KeyFrame(const size_t ID, const double time_stamp, const Mat34d &T_wo,
                   const Mat34d &T_wb, const laserCloud::Ptr cornerCloud,
                   const laserCloud::Ptr surfCloud,
                   const laserCloud::Ptr withoutdGroundCloud)
    : index_(ID),
      time_stamp_(time_stamp),
      T_wo_(T_wo),
      T_wb_(T_wb),
      has_gps_(false) {
  pos_.x = T_wb(0, 3);
  pos_.y = T_wb(1, 3);
  pos_.z = T_wb(2, 3);
  laser_odom_cov_ = 0.3 * Mat6d::Identity();
  map_cov_ = laser_odom_cov_;
  corner_cloud_.reset(new laserCloud());
  surf_cloud_.reset(new laserCloud());
  without_ground_cloud_.reset(new laserCloud());
  *corner_cloud_ = *cornerCloud;
  *surf_cloud_ = *surfCloud;
  *without_ground_cloud_ = *withoutdGroundCloud;
}

KeyFrame::KeyFrame(const size_t ID, const double time_stamp, const Mat34d &T_wo,
                   const Mat34d &T_wb, const laserCloud::Ptr cornerCloud,
                   const laserCloud::Ptr surfCloud,
                   const laserCloud::Ptr withoutdGroundCloud,
                   const Vec3d &gps_pos, const double gps_cov)
    : index_(ID),
      time_stamp_(time_stamp),
      T_wo_(T_wo),
      T_wb_(T_wb),
      T_wl_(T_wb),
      gps_pos_(gps_pos),
      gps_cov_(gps_cov),
      has_gps_(true) {
  T_wm_ = Mathbox::Identity34();
  pos_.x = T_wb(0, 3);
  pos_.y = T_wb(1, 3);
  pos_.z = T_wb(2, 3);
  laser_odom_cov_ = 0.3 * Mat6d::Identity();
  map_cov_ = laser_odom_cov_;
  corner_cloud_.reset(new laserCloud());
  surf_cloud_.reset(new laserCloud());
  without_ground_cloud_.reset(new laserCloud());
  *corner_cloud_ = *cornerCloud;
  *surf_cloud_ = *surfCloud;
  *without_ground_cloud_ = *withoutdGroundCloud;
}

KeyFrame::KeyFrame(const size_t ID, const double time_stamp, const Mat34d &T_wo,
                   const Mat34d &T_wb, const laserCloud::Ptr cornerCloud,
                   const laserCloud::Ptr surfCloud, const Vec3d &gps_pos,
                   const double gps_cov)
    : index_(ID),
      time_stamp_(time_stamp),
      T_wo_(T_wo),
      T_wb_(T_wb),
      T_wl_(T_wb),
      gps_pos_(gps_pos),
      gps_cov_(gps_cov),
      has_gps_(true) {
  T_wm_ = Mathbox::Identity34();
  pos_.x = T_wb(0, 3);
  pos_.y = T_wb(1, 3);
  pos_.z = T_wb(2, 3);
  laser_odom_cov_ = 0.3 * Mat6d::Identity();
  map_cov_ = laser_odom_cov_;
  corner_cloud_.reset(new laserCloud());
  surf_cloud_.reset(new laserCloud());
  *corner_cloud_ = *cornerCloud;
  *surf_cloud_ = *surfCloud;
}

KeyFrame::~KeyFrame() {}

}  // namespace cvte_lidar_slam