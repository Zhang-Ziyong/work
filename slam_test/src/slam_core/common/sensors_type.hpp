#ifndef _SENSROS_TYPE_HPP_
#define _SENSROS_TYPE_HPP_

#include "slam_math.hpp"
#include "pc_base.hpp"

namespace EIOLIDAR_SLAM {

// 进行变量转换
struct OdomMeasure {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OdomMeasure() {}
  OdomMeasure(const Mat34d &pose, const double time_stamp)
      : pose(pose), time_stamp(time_stamp) {}
  OdomMeasure(const Mat34d &pose, const Eigen::Quaterniond Q_gyr_only,
              const double time_stamp)
      : pose(pose), Q_gyr_only(Q_gyr_only), time_stamp(time_stamp) {}
  OdomMeasure(const Mat34d &pose, const double linear, const double angular,
              const double time_stamp)
      : pose(pose), linear(linear), angular(angular), time_stamp(time_stamp) {}
  Mat34d pose;
  // Eigen::Vector3d linear_velocity;
  // Eigen::Vector3d angular_velocity;
  double linear;
  double angular;
  Eigen::Quaterniond Q_gyr_only;
  double time_stamp;
};

struct ImuMeasure {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuMeasure(){};
  ImuMeasure(const Vec3d acc, const Vec3d gyr, const size_t count,
             const double time_stamp)
      : acc(acc), gyr(gyr), count(count), time_stamp(time_stamp) {}

  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_comp = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyr_comp = Eigen::Vector3d::Zero();

  double k = 0.004, k_adj = 0.004;
  double G = 9.805;
  size_t count = 0;
  double time_stamp = 0;
  unsigned char init_flag = 0;
  Eigen::Vector3d ba0 = Eigen::Vector3d(-0.118, 0.121, 0.0);
  Eigen::Vector3d bg0 = Eigen::Vector3d(0.0000, 0.0000, -0.0000);
  Eigen::Vector3d ba = Eigen::Vector3d(0.23054, -0.22046, -0.14313);
  Eigen::Vector3d bg = Eigen::Vector3d(0.00127, 0.00012, -0.00339);
  double roll = 0, pitch = 0, yaw = 0;  // rad
  double roll_acc = 0, pitch_acc = 0, yaw_acc = 0;
  double roll_gyr = 0, pitch_gyr = 0, yaw_gyr = 0;
  double yaw_corrected = 0;
  Eigen::Matrix3d Rwi = Eigen::Matrix3d::Identity();
  Eigen::Quaterniond Qwi = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond Qwi_gyr_only = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond Qwl = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond Qwl_gyr_only = Eigen::Quaterniond::Identity();

};  // end of class

}  // namespace EIOLIDAR_SLAM

#endif