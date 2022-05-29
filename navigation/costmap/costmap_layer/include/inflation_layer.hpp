/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file Inflation_layer.hpp
 *
 *@brief costmap inflation layer.
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-08
 ************************************************************************/
#ifndef __INFLATION_LAYER_HPP
#define __INFLATION_LAYER_HPP

#include <boost/smart_ptr.hpp>
#include <mutex>
#include <queue>
#include "layer.hpp"
namespace CVTE_BABOT {
/**
 * CellData
 * @brief
 * 用于存储膨胀层障碍物栅格的信息，只在膨胀层会使用
 *
 **/

class CellData {
 public:
  /**
   * CellData
   * @brief  构造cellData
   *
   * @param[in] d-两点的距离
   * @param[in] i-栅格点在costmap中的下标
   * @param[in] x-栅格点在costmap中的x坐标
   * @param[in] y-栅格点在costmap中的y坐标
   * @param[in] sx-距离当前栅格最近点的x坐标
   * @param[in] sy-距离当前栅格最近点的y坐标
   */
  CellData(const double &d, const unsigned int &i, const unsigned int &x,
           const unsigned int &y, const unsigned int &sx,
           const unsigned int &sy)
      : distance_(d), index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy) {}
  CellData() = delete;
  CellData &operator=(const CellData &obj) {
    if (this == &obj) {
      return *this;
    }
    distance_ = obj.distance_;
    index_ = obj.index_;
    x_ = obj.x_;
    y_ = obj.y_;
    src_x_ = obj.src_x_;
    src_y_ = obj.src_y_;
    return *this;
  }
  CellData(const CellData &obj) {
    distance_ = obj.distance_;
    index_ = obj.index_;
    x_ = obj.x_;
    y_ = obj.y_;
    src_x_ = obj.src_x_;
    src_y_ = obj.src_y_;
  }
  double distance_;             ///<最近栅格的距离
  unsigned int index_;          ///<当前栅格的在costmap的下标
  unsigned int x_, y_;          ///<距离当前栅格最近点
  unsigned int src_x_, src_y_;  ///<当前栅格
};

/**
 * operator<
 * @brief  对比两个cellData中的距离
 *
 * @param[in] a-一个cellData
 * @param[in] b-另一个cellData
 * @return true-a>b, false-a<b
 */
inline bool operator<(const CellData &a, const CellData &b) {
  return a.distance_ > b.distance_;
}

/**
 * InflationLayer
 * @brief
 * 1.膨胀层的具体实现
 *
 **/
class InflationLayer final : public Layer {
 public:
  friend class InflationLayerTester;
  InflationLayer();
  ~InflationLayer();

  InflationLayer &operator=(const InflationLayer &obj) = delete;
  InflationLayer(const InflationLayer &obj) = delete;

  /**
   * reset
   * @brief 重置
   *
   */
  virtual void reset() override { onInitialize(); }

  /**
   * matchSize
   * @brief
   * 使该层大小和父层大小一致
   *
   * */
  virtual void matchSize() override;

  /**
   *getParams
   *@brief
   *配置参数
   *
   * */
  virtual void getParams() override;

  /**
   * onInitialize
   * @brief  Inflation layer中的初始化
   *
   * @return true-初始成功，false-初始失败
   */
  virtual bool onInitialize(void) override;

  /**
   * computeCost
   * @brief  根据距离计算代价
   *
   * @param[in] distance-距离
   * @return cost-代价
   */
  unsigned char computeCost(const double &distance) const;

  /**
   * updateBounds
   * @brief
   * 根据机器人坐标以及层的大小，更新layer的边界
   *
   * @param[in] wp_robot_pose-包含机器人x，y坐标,单位m;角度，单位rad
   * @param[out] cb_costmap_bound-指示边界范围数据
   * @return true-更新成功，false-更新失败
   * */
  virtual bool updateBounds(const WorldmapPose &wp_robot_pose,
                            CostmapBound &cb_costmap_bound) override;
  /**
   * updateCosts
   * @brief
   * 更新costmap的代价值
   *
   * @param[in] master_grid-更新的costmap对象
   * @param[in] i_min_i-更新范围的最小x
   * @param[in] i_min_j-更新范围的最小x
   * @param[in] i_max_i-更新范围的最大y
   * @param[in] i_max_j-更新范围的最大y
   * @return true-更新成功，false-更新失败
   *
   * */
  virtual bool updateCosts(const std::shared_ptr<Costmap2d> master_grid,
                           const int &i_min_i, const int &i_min_j,
                           const int &i_max_i, const int &i_max_j) override;

  std::shared_ptr<Costmap2d> getLayerCostMap() override { return nullptr; }

  typedef std::shared_ptr<InflationLayer>
      InflationLayerPtr;  ///<声明一个智能指针

 protected:
  /**
   * onFootprintChanged
   * @brief
   * 通知机器人的足迹已发生变化
   *
   * */
  virtual void onFootprintChanged();

  std::shared_ptr<std::recursive_mutex> inflation_access_ =
      nullptr;  ///< 递归锁

 private:
  /**
   * computeCaches
   * @brief
   * 计算缓存
   *
   * */
  bool computeCaches();

  /**
   * computeCaches
   * @brief
   * 清空缓存
   *
   * */
  void deleteKernels();

  void inflate(const int &x, const int &y, Costmap2d &master_grid);

  /**
   * distanceLookup
   * @brief
   * 计算两点距离
   *
   * @param[in] dx-两点x距离
   * @param[in] dy-两点y距离
   * @return distance-两点距离
   * */
  inline double distanceLookup(const unsigned int &dx, const unsigned int &dy) {
    return cached_distances_[dx][dy];
  }

  /**
   * costLookup
   * @brief
   * 计算两点代价
   *
   * @param[in] dx-两点x距离
   * @param[in] dy-两点y距离
   * @return cost-两点代价
   * */
  inline unsigned char costLookup(const unsigned int &dx,
                                  const unsigned int &dy) {
    return cached_costs_[dx][dy];
  }

  /**
   * printCachesCost
   * @brief
   * 打印缓存的代价值
   *
   * */
  void printCachesCost(const unsigned int &cache_size);

  /**
   *cellDistance
   *@brief
   *将世界地图的距离转换成栅格地图距离
   *
   *@param[in] d_world_dist-世界地图距离，单位m
   *@return -costmap栅格地图距离，单位个
   * */
  unsigned int cellDistance(const double &world_dist);

  double resolution_;        ///< 分辨率
  bool need_reinflation_;    ///< 下一次是否重新膨胀
  double inflation_radius_;  ///< 膨胀半径
  double inscribed_radius_;  ///< 以机器人原心为起点的机器人半径（内切圆）
  double weight_;                              ///< 膨胀权重
  unsigned char **cached_costs_ = nullptr;     ///<缓存代价
  double **cached_distances_ = nullptr;        ///<缓存距离
  unsigned int cell_inflation_radius_;         ///< 栅格的膨胀半径
  unsigned int cached_cell_inflation_radius_;  ///< 缓存中栅格的膨胀半径
  CostmapBound last_bound_;                    ///< 膨胀层的边界
};
}  // namespace CVTE_BABOT

#endif