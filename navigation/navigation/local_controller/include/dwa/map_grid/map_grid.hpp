/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file map_grid.hpp
 *
 *@brief 表征地图的栅格，后续路径和评分都是基于此地图
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-10
 ************************************************************************/

#ifndef __MAP_GRID_HPP
#define __MAP_GRID_HPP

#include <memory>
#include <pose2d/pose2d.hpp>
#include <queue>
#include "map_cell.hpp"

namespace CVTE_BABOT {
class Costmap2d;
/**
* MapGrid
* @brief
*   表征地图的栅格，后续路径和评分都是基于此地图
* */

class MapGrid {
 public:
  MapGrid() = default;
  ~MapGrid() = default;

  /**
* MapGrid
* @brief
*   构造函数
*
* @param[in] size_x-初始化地图的长度
* @param[in] size_y-初始化地图的宽度
* */
  MapGrid(const unsigned int &size_x, const unsigned int &size_y);

  /**
* MapGrid
* @brief
*   拷贝构造函数
*
* @param[in] mg-复制值
* */
  MapGrid(const MapGrid &mg);

  MapGrid &operator=(const MapGrid &mg) {
    if (this == &mg) {
      return *this;
    }
    size_y_ = mg.size_y_;
    size_x_ = mg.size_x_;
    map_ = mg.map_;
    return *this;
  }

  inline MapCell &operator()(const unsigned int &x, const unsigned int &y) {
    return map_[size_x_ * y + x];
  }

  inline MapCell operator()(const unsigned int &x,
                            const unsigned int &y) const {
    return map_[size_x_ * y + x];
  }

  inline MapCell &getCell(const unsigned int &x, const unsigned int &y) {
    return map_[size_x_ * y + x];
  }

  /**
* resetPathDist
* @brief
*   将路径距离和目标点距离的标志位清除
* */
  void resetPathDist();

  /**
* commonInit
* @brief
*   初始化所有栅格
* */
  void commonInit();

  /**
* sizeCheck
* @brief
*   检查是否需要重新申请内存
*
* @param[in] size_x-期望长度
* @param[in] size_y-期望宽度
* */
  void sizeCheck(const unsigned int &size_x, const unsigned int &size_y);

  /**
* getIndex
* @brief
*   坐标值转一维索引
*
* @param[in] x-坐标点x值
* @param[in] y-坐标点y值
* */
  inline size_t getIndex(const int &x, const int &y) { return size_x_ * y + x; }

  /**
* obstacleCosts
* @brief
*   表示一个单元格在障碍物中
* */
  inline double obstacleCosts() { return map_.size(); }

  /**
* unreachableCellCosts
* @brief
*   表示单元格超出墙外无法到达
* */
  inline double unreachableCellCosts() { return map_.size() + 1; }

  /**
* updatePathCell
* @brief
*   用于在路径距离计算中更新单元格的距离
*
* @param[in] current_cell-当前的栅格
* @param[in] check_cell-需要被更新的单元格
* @param[in] costmap-代价地图指针
* @return true代表更新成功
* */
  bool updatePathCell(MapCell *current_cell, MapCell *check_cell,
                      const std::shared_ptr<Costmap2d> costmap);

  /**
* adjustPlanResolution
* @brief
*   调整路径的分辨率去适应costmap的栅格大小，通过线性变化来填充缺省值
*
* @param[in] global_plan_in-输入的全局路径
* @param[in] global_plan_out-调整之后的全局路径
* @param[in] resolution-路径点之间期望的分辨率大小
* */
  static void adjustPlanResolution(const std::vector<Pose2d> &global_plan_in,
                                   std::vector<Pose2d> &global_plan_out,
                                   const double &resolution);

  /**
  * setTargetCells
  * @brief
  *   根据全局路径，更新目标栅格点
  *
  * @param[in] dist_queue-路径点队列
  * @param[in] costmap-代价地图
  * */
  void setTargetCells(const std::shared_ptr<Costmap2d> costmap,
                      const std::vector<Pose2d> &global_plan);

  /**
  * setLocalGoal
  * @brief
  *   根据局部目标点，更新目标栅格点
  *
  * @param[in] dist_queue-路径点队列
  * @param[in] costmap-代价地图
  * */
  void setLocalGoal(const std::shared_ptr<Costmap2d> costmap,
                    const std::vector<Pose2d> &global_plan);

  unsigned int getSizeX() { return size_x_; }

  unsigned int getSizeY() { return size_y_; }

  /**
    * computeTargetDistance
    * @brief
    *   计算地图点相对于规划路径的距离
    *
    * @param[in] dist_queue-路径点队列
    * @param[in] costmap-代价地图
    * */
  void computeTargetDistance(std::queue<MapCell *> &dist_queue,
                             const std::shared_ptr<Costmap2d> costmap);

 private:
  double goal_x_ = 0.0;  ///< 目标点在世界坐标系下的x值
  double goal_y_ = 0.0;  ///< 目标点在世界坐标系下的y值

  unsigned int size_x_ = 0;  ///< 地图长度
  unsigned int size_y_ = 0;  ///< 地图宽度

  std::vector<MapCell> map_;  ///< 地图
  std::vector<MapCell> reset_map_;  ///< 清除用的地图
  std::vector<Pose2d> adjusted_global_plan_; ///< setTargetCells调整分辨率之后的全局规划路径
  std::vector<Pose2d> goal_adjusted_global_plan_; ///< setLocalGoal调整分辨率之后的全局规划路径
};

}  // namespace CVTE_BABOT

#endif  // __MAP_GRID_HPP
