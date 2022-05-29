/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file costmap_model.hpp
 *
 *@brief 保存costmap的地图表示
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-10
 ************************************************************************/

#ifndef __COSTMAP_MODEL_HPP
#define __COSTMAP_MODEL_HPP

#include "costmap_2d.hpp"
#include <array>
#include <memory>
#include <pose2d/pose2d.hpp>

namespace CVTE_BABOT {
class Pose2d;
class Costmap2d;

/**
 * CostmapModel
 * @brief 用栅格地图来表示costmap的格式
 **/

class CostmapModel {
 public:
  ~CostmapModel() = default;

  /**
     *getPtrInstance
     *@brief
     *获取当前的单例指针
     *
     **/
  static std::shared_ptr<CostmapModel> getPtrInstance();

  /**
* footprintCost
* @brief
*   检查一个路径点与机器人footprint之间的代价值
*
* @param[in] x-路径中某一点的x值
* @param[in] y-路径中某一点的y值
* @param[in] theta-路径中某一点的角度theta值
* @param[in] footprint_spec-机器人的footpr
* @param[in] inscribed_radius-内接圆半径
* @param[in] circumscribed_radius-规划半径
* @return 返回该路径点的代价值
* */
  double footprintCost(const double &x, const double &y, const double &theta,
                       const std::array<Pose2d, 4> &footprint_spec);

  /**
* footprintCost
* @brief
*   检查是否存在障碍物在机器人的footprint内
*
* @param[in] position-机器人在世界坐标系下的位置
* @param[in] footprint-机器人的footpr
* @param[in] inscribed_radius-内接圆半径
* @param[in] circumscribed_radius-规划半径
* @return 返回值为正的时候代表所有障碍物都在机器人footprint以外
* */
  double footprintCost(const Pose2d &position,
                       const std::array<Pose2d, 4> &footprint);

  /**
* lineCost
* @brief
*   将一条直线在地图上栅格化，并检查是否会发生碰撞
*
* @param[in] x0-直线起点x值
* @param[in] y0-直线起点y值
* @param[in] x1-直线终点x值
* @param[in] y1-直线终点y值
* @return 返回值为正的时候代表是一条合法的线段，否则相反
* */
  double lineCost(const int &x0, const int &y0, const int &x1, const int &y1);

  /**
* lineCost
* @brief
*   将一条直线在地图上栅格化，并检查是否会发生碰撞,如果碰撞,输出碰撞点
*
*
* @param[in] x0-直线起点x值
* @param[in] y0-直线起点y值
* @param[in] x1-直线终点x值
* @param[in] y1-直线终点y值
* @param[out] compact_point-有障碍的点
* @return 返回值为正的时候代表是一条合法的线段，否则相反
* */
  bool lineCost(const int &x0, const int &y0, const int &x1, const int &y1,
                CostmapPoint &compact_point);
  /**
* lineCostStrict
* @brief
*   将一条直线在地图上栅格化，并检查是否会发生碰撞，判断的方式比lineCost严格
*
* @param[in] x0-直线起点x值
* @param[in] y0-直线起点y值
* @param[in] x1-直线终点x值
* @param[in] y1-直线终点y值
* @return 返回值为正的时候代表是一条合法的线段，否则相反
* */
  double lineCostStrict(const int &x0, const int &y0, const int &x1,
                        const int &y1);

  /**
* pointCost
* @brief
*   检查一个点在costmap上的代价值
*
* @param[in] x-检查点x值
* @param[in] y-检查点y值
* @return 返回值为正的时候代表是一个合法的点，否则相反
* */
  double pointCost(const int &x, const int &y);

  /**
* setPtrCostmap
* @brief
*   设置更新后的costmap指针
*
* @param[in] ptr_costmap-新的costmap指针
* */
  void setPtrCostmap(std::shared_ptr<Costmap2d> ptr_costmap) {
    costmap_ = ptr_costmap;
  }

  /**
   * getCostmap
   * @brief
   *    获取costmap指针
   * @return 当前新的costmap指针
   * */
  std::shared_ptr<Costmap2d> getCostmap() { return costmap_; }

  /**
   * worldToMap
   * @brief
   *    对costmap中世界坐标转地图坐标系的转换
   * @param[in] w_x-世界坐标系的x值
   * @param[in] w_y-世界坐标系的y值
   * @param[out] m_x-地图坐标系的x值
   * @param[out] m_y-地图坐标系的y值
   * @return true代表转化成功
   * */
  bool worldToMap(const double &w_x, const double &w_y, unsigned int &m_x,
                  unsigned int &m_y);

  /**
   * mapToWorld
   * @brief
   *    对costmap中地图坐标系转世界坐标的转换
   * @param[in] m_x-地图坐标系的x值
   * @param[in] m_y-地图坐标系的y值
   * @param[out] w_x-世界坐标系的x值
   * @param[out] w_y-世界坐标系的y值
   * */
  void mapToWorld(const unsigned int &m_x, const unsigned int &m_y, double &w_x,
                  double &w_y);

  /**
   * getCost
   * @brief
   *    获取指定单元格的代价值
   * @param[in] ui_x-地图坐标系的x值
   * @param[in] ui_y-地图坐标系的y值
   * @return 坐标(ui_x, ui_y)中代价值
   * */
  inline unsigned char getCost(const unsigned int &ui_x,
                               const unsigned int &ui_y) {
    return costmap_->getCost(ui_x, ui_y);
  }

  /**
   * getResolution
   * @brief
   *    获取地图的分辨率
   * @return costmap的地图分辨率
   * */
  inline double getResolution() { return costmap_->getResolution(); }

  /**
   * getSizeInMetersX
   * @brief
   *    获取地图的长度
   * @return 地图的长，单位（米）
   * */
  inline double getSizeInMetersX() { return costmap_->getSizeInMetersX(); }

  /**
   * getSizeInMetersY
   * @brief
   *    获取地图的宽度
   * @return 地图的宽，单位（米）
   * */
  inline double getSizeInMetersY() { return costmap_->getSizeInMetersY(); }

  /**
   * getSizeInCellsX
   * @brief
   *    获取地图的长度
   * @return 地图的长，单位（栅格）
   * */
  inline unsigned int getSizeInCellsX() { return costmap_->getSizeInCellsX(); }

  /**
   * getSizeInCellsY
   * @brief
   *    获取地图的宽度
   * @return 地图的宽，单位（栅格）
   * */
  inline unsigned int getSizeInCellsY() { return costmap_->getSizeInCellsY(); }

 private:
  CostmapModel();
  CostmapModel(const CostmapModel &obj) = delete;
  CostmapModel &operator=(const CostmapModel &obj) = delete;

  u_int16_t ui_cell_distance_to_obs_ =
      5;  //计算一条线上无障碍的点时，选择的点离障碍的栅格数，车辆避让用

  static std::shared_ptr<CostmapModel> ptr_instance_;  ///< 获取costmap的单例

  std::shared_ptr<Costmap2d> costmap_;  ///< costmap类指针
};

}  // namespace CVTE_BABOT

#endif  // __COSTMAP_MODEL_HPP