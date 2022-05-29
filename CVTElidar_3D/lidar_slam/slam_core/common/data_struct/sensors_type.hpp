/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *

 * @FilePath: /src/cvte_lidar_slam/slam_core/common/data_struct/sensors_type.hpp
 * @brief: 
 * @ 
 * @
 * @author: Wei Zhang(zhangwei@cvte.com)
 * @version: v 1.0
 * @Date: 2020-07-14 16:53:30
 ************************************************************************/
/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *

 * @FilePath: /src/cvte_lidar_slam/slam_core/common/data_struct/sensors_type.hpp
 * @brief: 
 * @ 
 * @
 * @author: Wei Zhang(zhangwei@cvte.com)
 * @version: v 1.0
 * @Date: 2020-07-14 16:53:30
 ************************************************************************/
#ifndef SENSORS_TYPE_HPP_
#define SENSORS_TYPE_HPP_

#include "common/math_base/slam_math.hpp"
#include "common/data_struct/pc_base.hpp"

namespace cvte_lidar_slam {

/**
 * OdomMeasure
 * @brief 里程计测量类
 *
 **/
struct OdomMeasure {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OdomMeasure() {}
  OdomMeasure(const Mat34d &pose, const double time_stamp)
      : pose(pose), time_stamp(time_stamp) {}
  Mat34d pose;
  double time_stamp;
};  // end of class

/**
 * ImuMeasure
 * @brief imu测量类
 *
 **/  
struct ImuMeasure {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuMeasure() = delete;
  ImuMeasure(const Vec3d &acc, const Vec3d &gyro, const double time_stamp)
      : acc(acc), gyro(gyro), time_stamp(time_stamp) {}
  Vec3d acc;
  Vec3d gyro;
  double time_stamp;
}; // end of class

/**
 * GPSMeasure
 * @brief gps测量类
 *
 **/
struct GPSMeasure {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GPSMeasure(const Vec3d &nav_pos, const double time_stamp, const double cov,
            unsigned int loc_type)
      : nav_pos(nav_pos),
        time_stamp(time_stamp),
        cov(cov),
        loc_type(loc_type) {}
  Vec3d nav_pos;
  double time_stamp;
  double cov;
  unsigned int loc_type;
};  // end of class

/**
 * GpsMsg
 * @brief gps消息类
 *
 **/
typedef struct GpsMsg {
  GpsMsg() = default;
  GpsMsg(const double latitude, const double longitude, const double altitude, const double cov, const double time_stamp, const unsigned int loc_type) : 
    latitude(latitude), longitude(longitude), altitude(altitude), cov(cov), time_stamp(time_stamp), loc_type(loc_type) {}
  double latitude;        ///< 纬度
  double longitude;       ///< 经度
  double altitude;        ///< 高度
  double cov;             ///< 定位误差
  double time_stamp;       ///< 时间
  unsigned int loc_type;  ///< 定位解类型： 4-fixed, 5-float

} GpsMsg;  ///< 定位pose数据

/**
 * CloudMeasure
 * @brief cloud测量类
 *
 **/

struct CloudMeasure {
  CloudMeasure(const laserCloud::Ptr cloud_in, const double time_stamp)
      :cloud(cloud_in), time_stamp(time_stamp) {}
  CloudMeasure() = default;
  laserCloud::Ptr cloud;
  double time_stamp;
}; // end of class

/**
 * RobotState
 * @brief robot state类
 *
 **/

struct RobotState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RobotState() = default;
  RobotState(const Mat34d &pose, const Vec3d &speed, const double time_stamp, const double cov)
      :pose(pose), speed(speed), time_stamp(time_stamp),cov(cov) {}
  void Reset() {
    pose = Mat34d::Identity();
    speed = Vec3d::Zero();
    time_stamp = -1.;
    cov = -1.;
  }
  Mat34d pose;
  Vec3d speed;
  double time_stamp;
  double cov;
};  // end of class

struct KeyFrameInfo {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  KeyFrameInfo() = default;
  KeyFrameInfo(const size_t frame_id, const Mat34d& pose, const double pose_cov,
      const bool has_gps = false, const Vec3d& gps_pos = Vec3d::Zero(), const double gps_cov = -1 ):
      frame_id(frame_id), pose(pose), pose_cov(pose_cov), has_gps(has_gps),gps_pos(gps_pos),gps_cov(gps_cov) {}
  size_t frame_id;
  Mat34d pose;
  double pose_cov;
  bool has_gps;
  Vec3d gps_pos;
  double gps_cov;

}; // end of class



} // namespace cvte_lidar_slam

#endif // SENSORS_TYPE_HPP_