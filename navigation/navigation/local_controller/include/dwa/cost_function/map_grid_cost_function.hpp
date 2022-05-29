/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file map_grid_cost_function.hpp
 *
 *@brief DWA中计算地图的评价函数
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-09
 ************************************************************************/

#ifndef __MAP_GRID_COST_FUNCTION_HPP
#define __MAP_GRID_COST_FUNCTION_HPP

#include "cost_function.hpp"
#include "trajectory.hpp"

#include <pose2d/pose2d.hpp>

#include <memory>
#include <vector>

namespace CVTE_BABOT {

// 在通过栅格点计算路径总得分时，可以分为三种模式（最后一点/ 总和/ 相乘）
enum CostAggregationType { Last = 0, Sum = 1, Product = 2 };
class MapGrid;
class Costmap2d;

/**
* MapGridCostFunction
* @brief
*   DWA中计算地图的评价函数
* */

class MapGridCostFunction : public CostFunction {
 public:
  MapGridCostFunction() = delete;
  MapGridCostFunction(const MapGridCostFunction &obj);
  MapGridCostFunction &operator=(const MapGridCostFunction &obj);
  MapGridCostFunction(double xshift = 0.0, double yshift = 0.0,
                      bool is_local_goal_function = false,
                      CostAggregationType aggregationType = Last);

  ~MapGridCostFunction() = default;

  /**
* setTargetPoses
* @brief
*   设置目标点集合
*
* @param[in] target_poses-目标点集合
* */
  void setTargetPoses(const std::vector<Pose2d> &target_poses);

  /**
* prepare
* @brief
*   用来检查各个代价函数是否准备完成
* */
  bool prepare() override;

  /**
* scoreTrajectory
* @brief
*   计算一条路径的得分
*
* @param[in] traj-需要计算分数的路径
* */
  double scoreTrajectory(const Trajectory &traj) override;

  /**
* setXShit
* @brief
*   设置x轴的偏移量
*
* @param[in] x_shift-偏移量
* */
  inline void setXShift(const double &x_shift) { xshift_ = x_shift; }

  /**
* setYShit
* @brief
*   设置y轴的偏移量
*
* @param[in] y_shift-偏移量
* */
  inline void setYShift(const double &y_shift) { yshift_ = y_shift; }

  /**
* setStopOnFailure
* @brief
*   设置错误停止的标志位
*
* @param[in] stop_on_failure-是否需要错误停止
* */
  inline void setStopOnFailure(const bool &stop_on_failure) {
    stop_on_failure_ = stop_on_failure;
  }

  /**
* obstacleCosts
* @brief
*   获取对应障碍物的代价值
*
* @return 障碍物代价值
* */
  double obstacleCosts();

  /**
* unreachableCellCosts
* @brief
*   无法到达的栅格的代价值
*
* @return 代价值
* */
  double unreachableCellCosts();

  /**
* resetPathDist
* @brief
*   重置与路径距离的标志位
* */
  void resetPathDist();

  /**
* getCellCosts
* @brief
*   获取一个点的代价值
* @param[in] cx-坐标点x轴
* @param[in] cy-坐标点y轴
* @return 坐标点的代价值
* */
  double getCellCosts(const unsigned int &cx, const unsigned int &cy);

 private:
  std::vector<Pose2d> target_poses_;    ///< 采样的目标点

  std::shared_ptr<MapGrid> ptr_map_;     ///< 用于表示costmap的地图
  CostAggregationType aggregationType_;  ///< 计算得分总分的方法

  double xshift_ = 0.0;  ///< x轴的偏移
  double yshift_ = 0.0;  ///< y轴的偏移

  bool is_local_goal_function_ = false;  ///< 是否存在局部目标函数
  bool stop_on_failure_ = false;         ///< 采样失败时是否停止
};

}  // namespace CVTE_BABOT

#endif  // __MAP_GRID_COST_FUNCTION_HPP