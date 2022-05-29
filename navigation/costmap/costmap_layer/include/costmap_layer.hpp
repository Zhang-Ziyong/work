/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file costmap_layer.hpp
 *
 *@brief 继承Layer和costmap，是StaticLayer，ObstacleLayer,
 *RangeSensorLayer的基类
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified chenmingjian (chenmigjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-09
 ************************************************************************/
#ifndef __COSTMAP_LAYER_HPP
#define __COSTMAP_LAYER_HPP

#include "costmap_2d.hpp"
#include "layer.hpp"

namespace CVTE_BABOT {

/**
 * CostmapLayer
 * @brief
 * 1.构造StaticLayer，ObstacleLayer,RangeSensorLayer的基类
 * 2. 实现对用layer数据更新主costmap的操作
 *
 **/
class CostmapLayer : public Layer, public Costmap2d {
 public:
  CostmapLayer()
      : b_has_extra_bounds_(false),
        d_extra_min_x_(1e6),
        d_extra_max_x_(-1e6),
        d_extra_min_y_(1e6),
        d_extra_max_y_(-1e6) {}

  CostmapLayer(const CostmapLayer &ob) = delete;
  CostmapLayer &operator=(const CostmapLayer &ob) = delete;

  /**
   * matchSize
   * @brief
   * 使该层大小和父层大小一致
   *
   * */
  virtual void matchSize() override;

  /**
   * addExtraBounds
   * @brief
   * 增加外部资源的边界
   *
   * @param[in] d_mx0-外部边界的x轴最小值
   * @param[in] d_my0-外部边界的y轴最小值
   * @param[in] d_mx1-外部边界的x轴最大值
   * @param[in] d_mx1-外部边界的x轴最大值
   *
   * */
  void addExtraBounds(double &d_mx0, double &d_my0, double &d_mx1,
                      double &d_my1);

  /**
   *isConvexPolygonFill
   *@brief
   *判断机器人多边形内有无障碍物
   *
   *@param[in] polygon-机器人的多边形
   *@return true-机器人多边形内有障碍物，false-机器人多边形内无障碍物
   * */
  virtual bool isConvexPolygonFill(
      const std::vector<WorldmapPoint> &polygon) override;

 protected:
  /**
   * updateWithTrueOverwrite
   * @brief
   * 用当前层指定区域代价值的更新主costamp，所有值都要更新
   *
   * @param[in] ptr_master_grid-主costmap
   * @param[in] i_min_i-x轴最小值
   * @param[in] i_min_j-y轴最小值
   * @param[in] i_max_i-x轴最大值
   * @param[in] i_max_j-x轴最大值
   *
   * */
  void updateWithTrueOverwrite(const std::shared_ptr<Costmap2d> ptr_master_grid,
                               const int &i_min_i, const int &i_min_j,
                               const int &i_max_i, const int &i_max_j);

  /**
   * updateWithOverwrite
   * @brief
   * 用当前层指定区域代价值的更新主costamp，值为NO_INFORMATION 不用更新
   *
   * @param[in] ptr_master_grid-主costmap
   * @param[in] i_min_i-x轴最小值
   * @param[in] i_min_j-y轴最小值
   * @param[in] i_max_i-x轴最大值
   * @param[in] i_max_j-x轴最大值
   *
   * */
  void updateWithOverwrite(const std::shared_ptr<Costmap2d> ptr_master_grid,
                           const int &i_min_i, const int &i_min_j,
                           const int &i_max_i, const int &i_max_j);

  /**
   * updateWithMax
   * @brief
   * 用当前层指定区域代价值的更新主costamp，当主costmap为NO_INFORMATION需要更新
   * 当layer为NO_INFORMATION，主costmap不更新。
   *
   * @param[in] ptr_master_grid-主costmap
   * @param[in] i_min_i-x轴最小值
   * @param[in] i_min_j-y轴最小值
   * @param[in] i_max_i-x轴最大值
   * @param[in] i_max_j-x轴最大值
   *
   * */
  void updateWithMax(const std::shared_ptr<Costmap2d> ptr_master_grid,
                     const int &i_min_i, const int &i_min_j, const int &i_max_i,
                     const int &i_max_j);

  /**
   * updateWithAddition
   * @brief
   * 用当前层指定区域代价值的更新主costamp，当主costmap为NO_INFORMATION需要更新
   * 当layer为NO_INFORMATION，主costmap不更新。当总代价大于INSCRIBED_INFLATED_OBSTACLE，主costmap的值设置成
   * (INSCRIBED_INFLATED_OBSTACLE - 1)
   *
   * @param[in] ptr_master_grid-主costmap
   * @param[in] i_min_i-x轴最小值
   * @param[in] i_min_j-y轴最小值
   * @param[in] i_max_i-x轴最大值
   * @param[in] i_max_j-x轴最大值
   *
   * */
  void updateWithAddition(const std::shared_ptr<Costmap2d> ptr_master_grid,
                          const int &i_min_i, const int &i_min_j,
                          const int &i_max_i, const int &i_max_j);

  /**
   * touch
   * @brief
   * 根据坐标来更新边界
   *
   * @param[in] wm_point-x轴，y轴坐标
   * @param[out] cb_bound-要更新的地图范围
   *
   * */
  void touch(const WorldmapPoint &wm_point, CostmapBound &cb_bound);

  /**
   * touch
   * @brief
   * 使用额外的更新边界
   * @param[out] cb_costmap_bound-要更新的地图范围
   *
   * */
  void useExtraBounds(CostmapBound &cb_costmap_bound);

  void loadAvandanceBound();

  bool b_has_extra_bounds_;  ///< 是否使用外部资源边界
  std::shared_ptr<Costmap2d> getLayerCostMap() override {
    return std::make_shared<Costmap2d>(*this);
  }

 private:
  double d_extra_min_x_;  ///< 外部资源边界x最小
  double d_extra_max_x_;  ///< 外部资源边界x最大
  double d_extra_min_y_;  ///< 外部资源边界y最小
  double d_extra_max_y_;  ///< 外部资源边界y最大
};

}  // namespace CVTE_BABOT
#endif  // __COSTMAP_LAYER_HPP
