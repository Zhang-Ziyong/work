/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 201９, CVTE.
 * All rights reserved.
 *
 *@file occupancy_map_common.hpp
 *
 *@brief 占用栅格地图相关的类型定义
 *
 *
 *@author caoyong(caoyong@cvte.com)
 *@version 1.0
 *@data 2021-04-21
 ************************************************************************/
#ifndef SLAM_2D_OCCUPANCY_MAP_COMMON_HPP
#define SLAM_2D_OCCUPANCY_MAP_COMMON_HPP

#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/console/time.h>
#include <pcl/filters/extract_indices.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

namespace slam2d_core {
namespace occupancy_map {

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> laserCloud;
typedef Eigen::Matrix<double, 3, 4, Eigen::RowMajor> Mat34d;
typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Mat3d;
typedef Eigen::Vector2d Vec2d;
typedef Eigen::Vector3d Vec3d;

// const float mapCubeLength = 1.0;  // the length of a sub-map (meters)
// const float mapResolution = 0.05;
// const int mapCubeArrayLength = mapCubeLength / mapResolution;
// const int mapArrayLength = 2000 / mapCubeLength;
// const int rootCubeIndex = mapArrayLength / 2;

// // 2D Map Publish Params
// const int localMapLength =
//     20;  // length of the local occupancy grid map (meter)
// const int localMapArrayLength = localMapLength / mapResolution;
/**
 * ScanData
 * @brief 激光扫描数据结构
 *
 **/
struct ScanData {
  std::vector<float> v_laser_scan;  // laser scan data
  size_t frame_id;                  // 最邻近的id
  Mat34d delta_pose;           // 当前scan位置与最邻近的相对位置
  laserCloud::Ptr view_cloud;  // 可视化点云
};

/**
 * TraverCloudData
 * @brief 点云数据
 *
 **/
struct CloudData {
  laserCloud::Ptr laser_cloud;  // laser cloud data
  size_t frame_id;              // 最近关键帧的id
  Mat34d delta_pose;  // 当前点云位置与最近关键帧的相对位置
};

/**
 * OccMapConfig
 * @brief 2d 栅格地图参数
 **/
struct OccMapConfig {
  std::string occ_map_type = "scan";
  int camera_resolution_w = 160;
  float laser_min_angle =
      -M_PI * 0.18f;  // (34° = 180 × 0.18) ,相机深度FOV水平是68°
  float laser_max_angle = M_PI * 0.18f;
  float laser_min_range = 0.05;
  float laser_max_range = 10;
  float laser_angle_increment =
      1.0f / camera_resolution_w * M_PI * 0.36f;  // 0.36 = 0.18 * 2
  bool use_view_cloud = true;  // 使用点云增强地图细节

  std::vector<double> search_area = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float add_view_value = 0.4;
  float reduce_view_value = 0.4;

};  // end of class

/**
 * UserOccupancyGrid
 * @brief 自定义占用栅格数据类型
 *
 **/
struct UserOccupancyGrid {
  struct Header {
    std::string frame_id;
    double time_stamp = -1.;
  };  // header info
  struct MapInfo {
    float resolution;
    u_int32_t width;
    u_int32_t height;
    Mat34d origin;

  };  // MetaData for the map
  Header header;
  MapInfo info;
  std::vector<int8_t> data;
};  // end of class

/**
 * Grid
 * @brief 地图栅格数据结构
 *
 **/
struct Grid {
  int mapID;
  int cubeX;
  int cubeY;
  int gridX;
  int gridY;
  int gridIndex;
  float pointX;
  float pointY;
};  // end of class

/**
 * MapCell
 * @brief Cell Definition
 * 1.a cell is a member of a grid in a sub-map
 * 2.a grid can have several cells in it.
 * 3.a cell represent one height information
 **/
struct MapCell {
  float log_odds;
  float pointX;
  float pointY;

  MapCell() { log_odds = 0.; }

  inline float getBel() { return (1.0 - 1.0 / (1 + exp(log_odds))); }

  void setLogBel(const float log_bel) {
    log_odds += log_bel;
    if (log_odds > 5 || log_odds < -4) {
      log_odds -= log_bel;
    }
  }

  void setCurbLogBel(const float &log_bel, const float &max_bel_value) {
    log_odds += log_bel;
    // if (log_odds > 100 || log_odds < -4) {
    //   log_odds -= log_bel;
    // }
    if (log_odds > max_bel_value) {
      log_odds = max_bel_value;
    }
  }

  void setCurbDownLogBel(const float &log_bel) {
    log_odds -= log_bel;
    if (log_odds < 0) {
      log_odds = 0.0;
    }
  }
};  // end of class

/**
 * Sub-map Definition
 * @brief Cell Definition
 * 1.ChildMap is a small square. We call it "cellArray".
 * 2.It composes the whole map
 **/
struct ChildMap {
  std::vector<std::vector<MapCell>> cellArray;
  int subInd;     // sub-map's index in 1d mapArray
  int indX;       // sub-map's x index in 2d array mapArrayInd
  int indY;       // sub-map's y index in 2d array mapArrayInd
  float originX;  // sub-map's x root coordinate
  float originY;  // sub-map's y root coordinate
  // pcl::PointCloud<PointType> cloud;

  ChildMap(const int &id, const int &indx, const int &indy,
           const int &rootCubeIndex, const int &mapCubeLength,
           const int &mapCubeArrayLength) {
    subInd = id;
    indX = indx;
    indY = indy;
    originX = (indX - rootCubeIndex) * mapCubeLength - mapCubeLength / 2.0;
    originY = (indY - rootCubeIndex) * mapCubeLength - mapCubeLength / 2.0;

    // allocate and initialize each cell
    cellArray.resize(mapCubeArrayLength);
    for (int i = 0; i < mapCubeArrayLength; ++i)
      cellArray[i].resize(mapCubeArrayLength);

    for (int i = 0; i < mapCubeArrayLength; ++i)
      for (int j = 0; j < mapCubeArrayLength; ++j) cellArray[i][j] = MapCell();
    // allocate point cloud for visualization
  }
};  // end of class

/**
 * Pose2d
 * @brief 2d pose类
 *
 **/
class Pose2d {
 public:
  Pose2d() {
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
  }
  Pose2d(double x, double y, double theta) : x_(x), y_(y), theta_(theta) {}

  const Pose2d operator*(const Pose2d &p2) {
    Pose2d p;
    Eigen::Matrix2d R;
    R << cos(theta_), -sin(theta_), sin(theta_), cos(theta_);
    Vec2d pt2(p2.x_, p2.y_);
    Vec2d pt = R * pt2 + Vec2d(x_, y_);

    p.x_ = pt(0);
    p.y_ = pt(1);
    p.theta_ = theta_ + p2.theta_;
    NormAngle(p.theta_);
    return p;
  }

  const Vec2d operator*(const Vec2d &p) {
    Eigen::Matrix2d R;
    R << cos(theta_), -sin(theta_), sin(theta_), cos(theta_);
    Vec2d t(x_, y_);
    return R * p + t;
  }

  Pose2d inv() {
    double x = -(cos(theta_) * x_ + sin(theta_) * y_);
    double y = -(-sin(theta_) * x_ + cos(theta_) * y_);
    double theta = -theta_;
    return Pose2d(x, y, theta);
  }

  void NormAngle(double &angle) {
    while (angle >= M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
  }
  double x() { return x_; }
  double y() { return y_; }
  double yaw() { return theta_; }

 private:
  double x_, y_, theta_;
};  // class Pose2d

/**
 * Mathbox
 * @brief 基础运算类
 *
 **/
class Mathbox {
 public:
  /**
   *isRotationMatrix
   *@brief
   *判断矩阵是否是旋转矩阵
   *
   *@param[in] R-矩阵
   *@return bool
   **/
  static bool isRotationMatrix(const Mat3d &R) {
    Mat3d Rt;
    Rt = R.transpose();
    Mat3d shouldBeIdentity = Rt * R;
    Mat3d I = Mat3d::Identity();
    return (shouldBeIdentity - I).norm() < 2e-6;
  }

  static Vec3d rotationMatrixToEulerAngles(const Mat3d &R) {
    assert(isRotationMatrix(R));
    double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
    bool singular = sy < 1e-6;  // true: `Y`方向旋转为`+/-90`度
    double x, y, z;
    if (!singular) {
      x = atan2(R(2, 1), R(2, 2));
      y = atan2(-R(2, 0), sy);
      z = atan2(R(1, 0), R(0, 0));
    } else {
      x = atan2(-R(1, 2), R(1, 1));
      y = atan2(-R(2, 0), sy);
      z = 0;
    }
    return Vec3d(x, y, z);
  }

  static Pose2d Mat34d2Pose2d(const Mat34d &pose) {
    const double x = pose(0, 3);
    const double y = pose(1, 3);
    const Vec3d euler_angle =
        rotationMatrixToEulerAngles(pose.block<3, 3>(0, 0));
    const double theta = euler_angle[2];
    return Pose2d(x, y, theta);
  }

  /**
   *multiplePoint
   *@brief
   *对点进行坐标变换
   *
   *@param[in] pose
   *@param[in] point－坐标点
   *@return Vec3d
   **/
  static Vec3d multiplePoint(const Mat34d &pose, const Vec3d &point) {
    Vec3d ret = pose.block<3, 3>(0, 0) * point + pose.block<3, 1>(0, 3);
    return ret;
  }

  static Mat34d Identity34() {
    static Mat34d identity = Mat34d::Constant(std::nan(""));
    if (identity.hasNaN()) {
      identity.block<3, 3>(0, 0) = Mat3d::Identity();
      identity.block<3, 1>(0, 3) = Vec3d::Zero();
    }
    return identity;
  }

  /**
   *multiplePose34d
   *@brief
   *pose乘法运算
   *
   *@param[in] pose1
   *@param[in] pose2
   *@return Mat34d
   **/
  static Mat34d multiplePose34d(const Mat34d &pose1, const Mat34d &pose2) {
    Mat34d ret = Identity34();
    ret.block<3, 3>(0, 0) = pose1.block<3, 3>(0, 0) * pose2.block<3, 3>(0, 0);
    ret.block<3, 1>(0, 3) = pose1.block<3, 3>(0, 0) * pose2.block<3, 1>(0, 3) +
                            pose1.block<3, 1>(0, 3);
    return ret;
  }

  /**
   *inversePose34d
   *@brief
   *计算pose的逆
   *
   *@param[in] pose
   *@return Mat34d
   **/
  static Mat34d inversePose34d(const Mat34d &pose) {
    Mat34d ret = pose;
    ret.block<3, 3>(0, 0) = pose.block<3, 3>(0, 0).transpose();
    ret.block<3, 1>(0, 3) = -ret.block<3, 3>(0, 0) * pose.block<3, 1>(0, 3);
    return ret;
  }

  /**
   *deltaPose34d
   *@brief
   *计算相对pose
   *
   *@param[in] pose1
   *@param[in] pose2
   *@return Mat34d
   **/
  static Mat34d deltaPose34d(const Mat34d &pose1, const Mat34d &pose2) {
    return multiplePose34d(inversePose34d(pose1), pose2);
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

    return ypr;
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(
      const Eigen::MatrixBase<Derived> &ypr) {
    typedef typename Derived::Scalar Scalar_t;

    Scalar_t y = ypr(0);
    Scalar_t p = ypr(1);
    Scalar_t r = ypr(2);

    Eigen::Matrix<Scalar_t, 3, 3> Rz;
    Rz << cos(y), -sin(y), 0, sin(y), cos(y), 0, 0, 0, 1;

    Eigen::Matrix<Scalar_t, 3, 3> Ry;
    Ry << cos(p), 0., sin(p), 0., 1., 0., -sin(p), 0., cos(p);

    Eigen::Matrix<Scalar_t, 3, 3> Rx;
    Rx << 1., 0., 0., 0., cos(r), -sin(r), 0., sin(r), cos(r);

    return Rz * Ry * Rx;
  }

  static Eigen::Matrix3d g2R(const Eigen::Vector3d &g) {
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();
    Eigen::Vector3d ng2{0, 0, 1.0};
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    Eigen::Vector3d euler_angle = R2ypr(R0);
    // std::cout << "euler_angle " << euler_angle.transpose() << std::endl;
    double yaw = euler_angle.x();
    R0 = ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    // euler_angle = R2ypr(R0);
    // std::cout << "euler_angle " << euler_angle.transpose() << std::endl;
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
  }

  static Eigen::Matrix3d h2R(const Eigen::Vector3d &h) {
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = h.normalized();
    Eigen::Vector3d ng2{1.0, 0, 0};
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    Eigen::Vector3d euler_angle = R2ypr(R0);
    // std::cout << "euler_angle " << euler_angle.transpose() << std::endl;
    double roll = euler_angle.z();
    R0 = ypr2R(Eigen::Vector3d{0, 0, -roll}) * R0;
    // euler_angle = R2ypr(R0);
    // std::cout << "euler_angle " << euler_angle.transpose() << std::endl;
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
  }
  static Eigen::Vector3d h2Angles(const Eigen::Vector3d &h) {
    return R2ypr(h2R(h));
  }

  static Eigen::Vector3d g2Normal(const Eigen::Vector3d &g) {
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();
    Eigen::Vector3d ng2{0, 0, 1.0};
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    Eigen::Vector3d euler_angle = R2ypr(R0);
    double yaw = euler_angle.x();
    R0 = ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    return R2ypr(R0);
  }
};

class Transformbox {
 public:
  /**
   *pointAssociateToMap
   *@brief
   *将点变换到地图坐标系
   *
   *@param[in] pi-输入点
   *@param[in] pose-变换pose
   *@param[out] po-输出点
   **/
  static void pointAssociateToMap(PointType const *const pi,
                                  PointType *const po, const Mat34d &pose) {
    Vec3d point_curr(pi->x, pi->y, pi->z);
    Vec3d point_w = Mathbox::multiplePoint(pose, point_curr);
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
  }

  /**
   *transformPointCloud
   *@brief
   *将点云变换到地图坐标系
   *
   *@param[in] cloudIn-输入点云
   *@param[in] pose-变换pose
   *@return laserCloud::Ptr
   **/
  static laserCloud::Ptr transformPointCloud(const Mat34d &pose,
                                             laserCloud::Ptr cloudIn) {
    laserCloud::Ptr cloudOut(new laserCloud());
    PointType pointTo;

    int cloudSize = cloudIn->points.size();
    cloudOut->resize(cloudSize);
    for (int i = 0; i < cloudSize; ++i) {
      pointAssociateToMap(&cloudIn->points[i], &pointTo, pose);
      cloudOut->points[i] = pointTo;
    }
    return cloudOut;
  }

};  // end of class

}  // namespace occupancy_map
}  // namespace slam2d_core

#endif