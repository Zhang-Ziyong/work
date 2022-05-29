/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file point_sampler.hpp
 *
 *@brief hpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.3.0
 *@data 2020-09-17
 ************************************************************************/
#ifndef __POINT_SAMPLER_HPP
#define __POINT_SAMPLER_HPP
#include <utility>

#include "car_avoiding/car_avoiding_util.hpp"

namespace CVTE_BABOT {
const unsigned char DANGEROUS_COST = 150;
const double SAMPLE_RANGE_X = 3.5;
const double SAMPLE_RANGE_Y = 3.0;
const double SAMPLE_STEP_X = 0.5;

class PointSampler {
 public:
  PointSampler() = default;
  ~PointSampler() = default;

  PointSampler(const PointSampler &) = delete;
  PointSampler &operator=(const PointSampler &) = delete;

  /**
   *samplePoints
   *@brief
   *计算得到一些避让点供选择
   *
   *@param[in] costmap-costmap
   *@param[in] current_pose-机器人当前的位置
   *@param[in] car_pose-要避让的汽车的位置
   *@param[out] v_points-得到的避让点
  **/
  void samplePoints(const Costmap2d &costmap, const Pose2d &current_pose,
                    const Pose2d &car_pose,
                    std::vector<CostmapPoint> &v_sample_points);

 private:
  /**
  *getEndPoint
  *@brief
  *简介
  *
  *@param[in] origin_point-
  *@param[in] end_point-
  *@param[out] end_point_limited-
 **/
  void getEndPoint(const CostmapPoint &origin_point,
                   const CostmapPoint &end_point,
                   CostmapPoint &end_point_limited);

  /**
   *computeLineEndPoint
   *@brief
   *t
   *
   *@param[in] costmap-costmap
   *@param[in] v_line_points-
   *@param[in] v_end_points-
  **/
  void computeLineEndPoint(
      const Costmap2d &costmap,
      const std::vector<std::pair<CostmapPoint, CostmapPoint>> &v_line_points,
      std::vector<CostmapPoint> &v_end_points);

  /**
   *generateRayRight
   *@brief
   *生成一些从道路中点到路右边的射线, 用于从射线上获取邻近路右边的点
   *
   *@param[in] costmap-costmap
   *@param[in] road_pose-v_center_points
   *@param[out] v_center_points-道路的起点,
   *@param[out] v_right_points-射线的终点
  **/
  void generateLineRight(
      const Costmap2d &costmap, const Pose2d &road_pose,
      std::vector<std::pair<CostmapPoint, CostmapPoint>> &vp_line_points);
};
}  // namespace CVTE_BABOT
#endif
