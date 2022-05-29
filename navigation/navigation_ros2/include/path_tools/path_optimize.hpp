/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file path_optimize.hpp
 *
 *@brief
 * 对录制路径进行优化的类（针对来回前进回退一类的优化）
 *
 *@modified by jiajun liang(linjiajun@cvte.com)
 *
 *@author jiajun liang(linjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2019-07-15
 ************************************************************************/

#ifndef _PATH_OPTIMIZE_HPP_
#define _PATH_OPTIMIZE_HPP_

#include "pose2d/pose2d.hpp"
#include <vector>

namespace CVTE_BABOT {

struct CrossPoints {
  int origin_pose_index;            ///< 圆心在原始路径的索引
  int last_pose_index;              ///< 最后一个交点的索引
  Pose2d origin_pose;               ///< 以该点为圆心的位置
  std::vector<Pose2d> cross_points; ///< 与圆相交的点集合
};

/**
 * PathOptimize
 * @brief 处理路径优化的类
 **/

class PathOptimize {
public:
  /**
   *optimizeMissionPath
   *@brief
   * 对外开放的路径优化接口，将一条可能存在前后折返路径作为输入，
   * 输出一条将折线去掉的路径
   *
   *@param[in] origin_path - 需要进行路径优化的输入
   *@param[out] optimize_path - 最终优化后的路径结果
   *return: bool类型，true代表优化成功，false为失败
   * */
  bool optimizeMissionPath(const std::vector<Pose2d> &origin_path,
                           std::vector<Pose2d> &optimize_path);

  /**
   *setCrycleRadius
   *@brief
   * 设置优化圆的半径
   *
   *@param[in] r - 设置的半径大小
   * */
  void setCrycleRadius(const double &r);

  /**
   *setPathResolution
   *@brief
   * 设置输入路径的分辨率大小
   *
   *@param[in] l - 路径的分辨率
   * */
  void setPathResolution(const double &l);

  /**
   *isOnArcEdge
   *@brief
   * 判断一个路径点是否在圆弧边上
   *
   *@param[in] origin - 圆点
   *@param[in] side_point - 需要检查的路径点
   *@return - true代表检查点在弧边上，否则不在
   * */
  bool isOnArcEdge(const Pose2d &origin, const Pose2d &side_point);

  /**
   *findUglyPose
   *@brief
   * 查找路径上存在折线的位置
   *
   *@param[in] origin_path - 原始路径
   *@param[out] cross_points - 查找到的交叉点和相关索引
   *@return - true代表存在折线位置
   * */
  bool findUglyPose(const std::vector<Pose2d> &origin_path,
                    CrossPoints &cross_points);

  /**
   *twoPointDistance
   *@brief
   * 计算两点之间的直线距离
   *
   *@param[in] point_a - 点A
   *@param[in] point_b - 点B
   *@return - 点A和点B之间的直线距离（米）
   * */
  double twoPointDistance(const Pose2d &point_a, const Pose2d &point_b);

  /**
   *createOptimizePath
   *@brief
   * 在两点之间通过直线插值的方式生成一条直线路径
   *
   *@param[in] points - 查找到的交叉点信息
   *@param[in] result - 插值得到的直线结果
   *@return - true代表直线插值成功，false代表失败
   * */
  bool createOptimizePath(const CrossPoints &points,
                          std::vector<Pose2d> &result);

private:
  double optimize_radius_ = 0.5;  ///< 优化圆的半径
  double path_resolution_ = 0.25; ///< 路径分辨率
  int iterator_times_ = 0;        ///< 记录优化次数，避免无限迭代
};
} // namespace CVTE_BABOT

#endif // _PATH_OPTIMIZE_HPP_