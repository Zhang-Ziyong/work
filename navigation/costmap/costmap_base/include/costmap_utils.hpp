/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file costmap_utils.hpp
 *
 *@brief costmap_utils.
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-03
 ************************************************************************/
#ifndef __COSTMAP_UTILS_HPP
#define __COSTMAP_UTILS_HPP

#include <math.h>
#include <memory.h>
#include <algorithm>
#include <boost/shared_array.hpp>
#include <deque>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace CVTE_BABOT {

static const unsigned char NO_INFORMATION = 255;
static const unsigned char LETHAL_OBSTACLE = 254;
static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static const unsigned char FREE_SPACE = 0;

static const unsigned int FIELD_NUM = 19 * 11;

static const unsigned int OBSTACLE_DIREC_AREA = 6;

struct CostmapPoint {
  CostmapPoint() = default;
  CostmapPoint(unsigned int _ui_x, unsigned int _ui_y)
      : ui_x(_ui_x), ui_y(_ui_y) {}
  unsigned int ui_x = 0;
  unsigned int ui_y = 0;
};

struct WorldmapPoint {
  WorldmapPoint() {}
  WorldmapPoint(double x, double y) : d_x(x), d_y(y) {}
  double d_x = 0.000;
  double d_y = 0.000;
};

struct VoxelPoint {
  unsigned int ui_x = 0;
  unsigned int ui_y = 0;
  unsigned int ui_z = 0;
};

struct WorldmapPose {
  double d_x = 0.000;
  double d_y = 0.000;
  double d_yaw = 0.000;
};

struct SensorPose {
  double x = 0.000;
  double y = 0.000;
  double z = 0.000;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

struct WorldmapPoseWithTime {
  double d_x = 0.000;
  double d_y = 0.000;
  double d_yaw = 0.000;
  double d_time = 0.000;
};

struct CostmapBound {
  double d_min_x = 0.000;
  double d_min_y = 0.000;
  double d_max_x = 0.000;
  double d_max_y = 0.000;
};

struct CostmapPointXYZ {
  double d_x = 0.000;
  double d_y = 0.000;
  double d_z = 0.000;
};

typedef std::vector<CostmapPointXYZ> CostmapPointCloud;

enum {
  StaticLayerType = 1,
  ObstacleLayerType,
  VoxelLayerType,
  ProbabilityVoxelLayerType,
  RangeSensorLayerType,
  InflationLayerType,
  NegativeObstaclesLayerType,
  CollisionLayerType,
  RangeSensorLayer2Type
};

enum SpeedLevel {
  Stop = -1,
  None = 0,
  SpeedOne,
  SpeedTwo,
  SpeedThree,
  SpeedMax
};
inline SpeedLevel operator+(SpeedLevel &aa, int a) {
  int temp = static_cast<int>(aa);
  temp = temp + a;
  if (static_cast<SpeedLevel>(temp) >= SpeedMax) {
    return SpeedMax;
  } else {
    return static_cast<SpeedLevel>(temp);
  }
}

enum AreaBOUND { LOW = 0, HIGH };
enum AreaDIREC {
  FRONT = 0,
  BACK,
  LEFT_FRONT,
  RIGHT_FRONT,
  LEFT_BACK,
  RIGHT_BACK
};
enum RobotType { FRONT_DRIVE = 0, BACK_DRIVE, CENTER_DRIVE };

struct LayerParammeter {
  void addLayer(int type, const std::string &s_name) {
    v_pair.push_back({type, s_name});
  }
  std::vector<std::pair<int, std::string>> v_pair;
};

/**
 *distance
 *@brief
 *计算两点距离
 *
 *@param[in] d_x0-点0的x
 *@param[in] d_y0-点0的y
 *@param[in] d_x1-点1的x
 *@param[in] d_y1-点1的y

 *@return distance-两点距离
 * */
inline double distance(const double &d_x0, const double &d_y0,
                       const double &d_x1, const double &d_y1) {
  return hypot(d_x1 - d_x0, d_y1 - d_y0);
}

/**
 *distanceToLine
 *@brief
 *点到直线的距离
 *
 *@param[in] pX-点p的x
 *@param[in] pY-点p的y
 *@param[in] x0-直线点0的x
 *@param[in] y0-直线点0的y
 *@param[in] x1-直线点1的x
 *@param[in] y1-直线点1的y

 *@return distance-点到直线距离
 * */
double distanceToLine(const double &pX, const double &pY, const double &x0,
                      const double &y0, const double &x1, const double &y1);

/**
 *distanceToLine
 *@brief
 *计算点集里两点的最大距离和最小距离
 *
 *@param[in] v_footprint-点集
 *@param[out] d_min_dist-最小距离
 *@param[out] d_max_dist-最小距离
 * */

void calculateMinAndMaxDistances(const std::vector<WorldmapPoint> &v_footprint,
                                 double &d_min_dist, double &d_max_dist);

/**
 *operator+
 *@brief
 *两点相加
 *
 *@param[in] wp_l-l点
 *@param[in] wp_r-r点
 *@return point-相加后的点
 * */
inline WorldmapPoint operator+(const WorldmapPoint &wp_l,
                               const WorldmapPoint &wp_r) {
  return {wp_l.d_x + wp_r.d_x, wp_l.d_y + wp_r.d_y};
}

/**
 *operator-
 *@brief
 *两点偏差
 *
 *@param[in] wp_l-l点
 *@param[in] wp_r-r点
 *@return point-偏差
 * */
inline WorldmapPoint operator-(const WorldmapPoint &wp_l,
                               const WorldmapPoint &wp_r) {
  return {wp_l.d_x - wp_r.d_x, wp_l.d_y - wp_r.d_y};
}

void transformFootprint(const WorldmapPose &wp_robot_pose,
                        const std::vector<WorldmapPoint> &v_wp_footprint,
                        std::vector<WorldmapPoint> &v_wp_transformed_footprint);

template <typename D>
void medianFiltering(std::deque<D> &q, D &data) {
  q.push_back(data);
  q.pop_front();
  std::deque<D> q2 = std::deque<D>(q);
  if (q2.size() != 0) {
    sort(q2.begin(), q2.end());
    data = q2.at(q2.size() / 2);
  }
}

boost::shared_array<bool> makeFiled();

void resetFiled(const boost::shared_array<bool> &ptr_field);

}  // namespace CVTE_BABOT

#endif
