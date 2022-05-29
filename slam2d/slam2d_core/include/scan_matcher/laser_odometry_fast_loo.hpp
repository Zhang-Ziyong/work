#ifndef _LASER_ODOMETRY_FAST_LOO_H
#define _LASER_ODOMETRY_FAST_LOO_H

// std header
#include <fstream>
#include <iostream>
#include <numeric>
#include <mutex>

// slam2d header
#include "amcl/laser_model.hpp"
#include "common/odometry_data.hpp"
#include "common/time.hpp"

// Eigen headers
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

// pcl headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/ndt_2d.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
// #include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZI PointType;

namespace slam2d_core {
namespace scan_matcher {
namespace laser_odometry_fast_loo {

struct LaserOdometryFastLooOption {
  unsigned int local_map_windows_size = 20;
  float key_frame_distance = 0.2;
  float key_frame_angle = 1.2;
  unsigned int frame_count = 8;

  // icp
  float icp_max_correspondence_distance = 0.4;
  unsigned int icp_max_num_iterations = 5;
  float icp_transformation_epsilon = 1e-6;
  float icp_euclidean_fitness_epsilon = 1e-6;
  float icp_fitness_score = 0.2;
  float icp_align_distance_threshold = 0.2;

  // icp down filter
  float icp_filter_leafsize = 0.1;
  float local_map_filter_leafsize = 0.1;

  // scan data
  float laser_min_range = 0.3;
  float laser_max_range = 30.0;

  // laser_tf
  std::vector<double> laser_tf = {0.0, 0.0, 0.0};

  // laser odom curr value
  double laserodom_correct_threshold = 0.3;
};

class Utility {
 public:
  template <typename Derived>
  static Eigen::Quaternion<typename Derived::Scalar> deltaQ(
      const Eigen::MatrixBase<Derived> &theta) {
    typedef typename Derived::Scalar Scalar_t;

    Eigen::Quaternion<Scalar_t> dq;
    Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
    half_theta /= static_cast<Scalar_t>(2.0);
    dq.w() = static_cast<Scalar_t>(1.0);
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    return dq;
  }

  static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R) {
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r =
        atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr / M_PI * 180.0;
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(
      const Eigen::MatrixBase<Derived> &ypr) {
    typedef typename Derived::Scalar Scalar_t;

    Scalar_t y = ypr(0) / 180.0 * M_PI;
    Scalar_t p = ypr(1) / 180.0 * M_PI;
    Scalar_t r = ypr(2) / 180.0 * M_PI;

    Eigen::Matrix<Scalar_t, 3, 3> Rz;
    Rz << cos(y), -sin(y), 0, sin(y), cos(y), 0, 0, 0, 1;

    Eigen::Matrix<Scalar_t, 3, 3> Ry;
    Ry << cos(p), 0., sin(p), 0., 1., 0., -sin(p), 0., cos(p);

    Eigen::Matrix<Scalar_t, 3, 3> Rx;
    Rx << 1., 0., 0., 0., cos(r), -sin(r), 0., sin(r), cos(r);

    return Rz * Ry * Rx;
  }

  // Eigen::Matrix3d g2R(const Eigen::Vector3d &g)
  // {
  //     Eigen::Matrix3d R0;
  //     Eigen::Vector3d ng1 = g.normalized();
  //     Eigen::Vector3d ng2{0, 0, 1.0};
  //     R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
  //     // 获得一个旋转矩阵把坐标ng1转换到ng2 double yaw = R2ypr(R0).x(); //
  //     取出计算的yaw角 R0 = ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0; //
  //     叠加一个负的yaw角，使旋转矩阵中的yaw角为0，即只有横滚俯仰角； return
  //     R0;  //
  //     R0是从当前坐标系到世界坐标系的旋转矩阵（当前坐标系的z轴，与世界坐标系的重力g的方向对应）
  // }

  template <typename T>
  static T normalizeAngle(const T &angle_degrees) {
    T two_pi(2.0 * 180);
    if (angle_degrees > 0)
      return angle_degrees -
             two_pi * std::floor((angle_degrees + T(180)) / two_pi);
    else
      return angle_degrees +
             two_pi * std::floor((-angle_degrees + T(180)) / two_pi);
  }
};

class TicToc {
 public:
  TicToc() { tic(); }

  void tic() { start = std::chrono::system_clock::now(); }

  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000;
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};

class LaserOdomtryFastLoo {
 public:
  LaserOdomtryFastLoo(const slam2d_core::scan_matcher::laser_odometry_fast_loo::
                          LaserOdometryFastLooOption &options);
  LaserOdomtryFastLoo(const LaserOdomtryFastLoo &) = delete;
  LaserOdomtryFastLoo &operator=(const LaserOdomtryFastLoo &) = delete;
  virtual ~LaserOdomtryFastLoo() = default;

  void odometryBufHandler(
      const slam2d_core::common::OdometryData &odometry_data);
  void laserScanBufHandler(const slam2d_core::amcl::LaserScanData &scan_data);
  void laserOdomProcess();
  bool getUpdateOdomPose(
      slam2d_core::common::OdometryData &odometry_data_update);

  bool laser_odom_init_finished_ = false;
  bool obtained_laser_odom_data_ = false;

  slam2d_core::common::OdometryData laser_odom_data_;
  std::mutex laser_odom_data_mutex_;

  // obtained scan cloud
  inline void getLaserOdomCloud(
      pcl::PointCloud<PointType>::Ptr &pre_aligned_laser_cloud,
      pcl::PointCloud<PointType>::Ptr &local_laser_cloud_map) {
    pre_aligned_laser_cloud = pub_pre_aligned_laser_cloud_;
    local_laser_cloud_map = pub_local_laser_cloud_map_;

    if (pub_pre_aligned_laser_cloud_ == nullptr) {
      LOG(ERROR) << "pub_pre_aligned_laser_cloud_ == nullptr";
    }
    if (pub_local_laser_cloud_map_ == nullptr) {
      LOG(ERROR) << "pub_local_laser_cloud_map_ == nullptr";
    }
  }

 private:
  void allocateMemory();
  void icpConfig();
  void init();

  bool isKeyFrame();
  void Quaternion2EulerAngle(const Eigen::Quaterniond &q, double &roll,
                             double &pitch, double &yaw);
  void downsampleCurrentLaserCloud();
  void lidarOnlyPredict();
  void transformUpdate();
  void localMapUpdate(const pcl::PointCloud<PointType>::Ptr &laserCloudTemp,
                      const Eigen::Vector3d &p, const Eigen::Quaterniond &q);
  void judgeDegradation();

 private:
  LaserOdometryFastLooOption options_;
  // wheel odom data
  std::mutex odom_buf_mutex_;
  std::queue<std::pair<double, Eigen::Isometry3d>> odom_buf_;
  double time_odom_curr_ = 0;
  // unsigned int frame_count_ = 0;

  Eigen::Vector3d dt_odom_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond dq_odom_ = Eigen::Quaterniond::Identity();

  Eigen::Vector3d dt_odom_last_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond dq_odom_last_ = Eigen::Quaterniond::Identity();

  Eigen::Vector3d dt_odom_last_keyframe_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond dq_odom_last_keyframe_ = Eigen::Quaterniond::Identity();

  // laser data
  std::mutex laser_buf_mutex_;
  std::queue<std::pair<double, pcl::PointCloud<PointType>::Ptr>> laser_buf_;
  double time_laser_curr_ = 0;
  unsigned int laser_cloud_downsample_size_ = 0;

  pcl::PointCloud<PointType>::Ptr laser_cloud_raw_ = nullptr,
                                  laser_cloud_downsample_ = nullptr,
                                  pre_aligned_laser_cloud_ = nullptr,
                                  local_laser_cloud_map_ = nullptr;

  pcl::PointCloud<PointType>::Ptr pub_pre_aligned_laser_cloud_ = nullptr,
                                  pub_local_laser_cloud_map_ = nullptr;
  pcl::PointCloud<PointType>::Ptr local_key_frame_cloud_[20];

  // icp
  pcl::VoxelGrid<PointType> down_size_filter_map_;
  pcl::VoxelGrid<PointType> down_size_filter_icp_;
  // pcl::IterativeClosestPoint<PointType, PointType> icp_tmp_;

  // init data
  Eigen::Quaterniond q_wmap_wodom_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t_wmap_wodom_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_wodom_curr_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t_wodom_curr_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_wodom_last_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t_wodom_last_ = Eigen::Vector3d::Zero();

  Eigen::Quaterniond q_w_curr_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t_w_curr_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_w_last_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t_w_last_ = Eigen::Vector3d::Zero();

  // smooth data
  Eigen::Quaterniond smooth_last_q_w_odom_new_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d smooth_last_t_w_odom_new_ = Eigen::Vector3d::Zero();
  bool first_odom_ = true;

  // init finish
  // bool laser_odom_init_finished_ = false;

  bool first_laser_odom_process_flag_ = true;

  // map data
  unsigned int local_key_frame_id_ = 0;

  // localization data
  Eigen::Isometry3f T_w_init_ = Eigen::Isometry3f::Identity();
  Eigen::Isometry3f T_w_last_ = Eigen::Isometry3f::Identity();
  Eigen::Isometry3f T_w_updated_ = Eigen::Isometry3f::Identity();

  Eigen::Isometry3f drt_odom_ = Eigen::Isometry3f::Identity();

  Eigen::Quaterniond q_laser;
  Eigen::Vector3d t_laser;

  Eigen::Quaterniond q_rl_, q_lr_;
  Eigen::Vector3d t_rl_, t_lr_;
  Eigen::Matrix3d R_rl_, R_lr_;
  Eigen::Matrix4d T_rl_, T_lr_;

  // judge degradation
  double judge_degradation_value_ = 8.0;
  bool is_degradation_ = false;
};
}  // namespace laser_odometry_fast_loo
}  // namespace scan_matcher
}  // namespace slam2d_core

#endif  // !_LASER_ODOMETRY_FAST_LOO_H