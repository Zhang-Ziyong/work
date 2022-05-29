/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file D_star.hpp
 *
 *@brief D*算法的头文件函数
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version Navigation-v2.0
 *@data 2019-12-16
 ************************************************************************/

#ifndef __D_STAR_HPP
#define __D_STAR_HPP

#include "expander.hpp"
#include "abstract_planner.hpp"
#include <boost/smart_ptr.hpp>
#include <iostream>

namespace CVTE_BABOT {
const int PRIORITYBUFSIZE = 10000;   ///< 优先级队列大小
const float INVSQRT2 = 0.707106781;  ///< 根号2
class PotentialCalculator;

/**
 * DijkstraExpansion
 * @brief
 *   D星算法的实现接口
 * */

class DijkstraExpansion : public Expander /*, public AbstractPlannerBase*/ {
  friend class DstarPlannerTest;

 public:
  DijkstraExpansion(const unsigned int &nx, const unsigned int &ny,
                    bool distance_map = false);
  DijkstraExpansion() = delete;
  ~DijkstraExpansion() = default;

  /**
   * calculatePotentials
   * @brief
   *   重载基类的实现，根据起点和目标点更新势场值大小
   *
   * @param[in] costs-代价地图
   * @param[in] start_x-起点的x坐标值
   * @param[in] start_y-起点的y坐标值
   * @param[in] end_x-终点的x坐标值
   * @param[in] end_y-终点的y坐标值
   * @param[in] cycles-循环次数
   *
   * @return true - 代表搜索到路径 ，否则相反
   * */
  bool calculatePotentials(const boost::shared_array<unsigned char> &costs,
                           const double &start_x, const double &start_y,
                           const double &end_x, const double &end_y,
                           const int &cycles) override;
  bool calculateAllPotentials(const boost::shared_array<unsigned char> &costs,
                              const double &start_x, const double &start_y,
                              const double &end_x, const double &end_y,
                              const int &cycles, int lethal_cost);
  void calculateAllPotentials(const boost::shared_array<unsigned char> &costs,
                              const double &start_x, const double &start_y,
                              const int &cycles, int lethal_cost);

  /**
   * setNeutralCost
   * @brief
   *   设置中立的代价值
   *
   * @param[in] neutral_cost-中立的代价值大小
   *
   * */
  inline void setNeutralCost(const unsigned char &neutral_cost) override;

  /**
   * setPreciseStart
   * @brief
   *   设置是否使用精确值更新势场值
   *
   * @param[in] precise-true/false 代表使用或不使用精确值计算
   *
   * */
  void setPreciseStart(const bool &precise) override { precise_ = precise; }

  /**
   * getCost
   * @brief
   *   获取某一点的代价值
   *
   * @param[in] costs-代价地图
   * @param[in] n-二维坐标对应一维的索引点
   * @return n点坐标的代价值
   * */
  float getCost(const boost::shared_array<unsigned char> &costs,
                const unsigned int &n);

  /**
   * updateCell
   * @brief
   *   更新某一点的代价值
   *
   * @param[in] costs-代价地图
   * @param[in] n-二维坐标对应一维的索引点
   * */
  void updateCell(const boost::shared_array<unsigned char> &costs, int n);

  /**
   * outlineMap
   * @brief
   *   将costmap的边界设置为致命，防止越界搜索
   *
   * @param[in] costs-代价地图
   * @param[in] x_size-代价地图的长
   * @param[in] y_size-代价地图的宽
   * @param[in] value-致命代价值
   * */
  bool outlineMap(boost::shared_array<unsigned char> costmap,
                  const unsigned int &x_size, const unsigned int &y_size,
                  const unsigned char &value);

  /**
   * pushCur
   * @brief
   *   将当前位置插入优先队列
   *
   * @param[in] n-二维坐标对应一维的索引点
   * */
  void pushCur(boost::shared_array<unsigned char> costs, const unsigned int &n);

  /**
   * pushNext
   * @brief
   *   将下一位置插入优先队列
   *
   * @param[in] n-二维坐标对应一维的索引点
   * */
  void pushNext(boost::shared_array<unsigned char> costs,
                const unsigned int &n);

  /**
   * pushOver
   * @brief
   *   将上一位置插入优先队列
   *
   * @param[in] n-二维坐标对应一维的索引点
   * */
  void pushOver(boost::shared_array<unsigned char> costs,
                const unsigned int &n);

  /**
   * makePlan
   * @brief
   *   输入一张地图，根据起点和终点规划出一条最短路径
   *
   * @param[in] costs-输入的代价地图
   * @param[in] start_x-起点x值
   * @param[in] start_y-起点y值
   * @param[in] end_x-终点x值
   * @param[in] end_y-终点y值
   * @param[out] n-二维坐标对应一维的索引点
   * @ return true-代表规划路径成功
   * */
  bool makePlan(boost::shared_array<unsigned char> &costs,
                const Pose2d &start_pose, const Pose2d &end_pose,
                std::vector<Pose2d> &path_result);

  /**
   * getDistancePotential
   * @brief
   *   获取指定位置的距离势场值，可用于启发式搜索
   *
   * @param[in] pose_x-查询点的x坐标值（图像坐标系）
   * @param[in] pose_y-查询点的y坐标值（图像坐标系）
   * @ return 距离势场值
   * */
  float getDistancePotential(int pose_x, int pose_y);

 private:
  /**
   * setSize
   * @brief
   *   重新设置势场的地图大小
   *
   * @param[in] nx-地图x轴长度
   * @param[in] ny-地图y轴长度
   * */
  void setSize(const unsigned int &nx, const unsigned int &ny) override;

  boost::shared_array<int> currentBuffer_;  ///< 优先级队列的块
  boost::shared_array<int> nextBuffer_;
  boost::shared_array<int> overBuffer_;

  boost::shared_array<bool> pending_;  ///< 记录未访问的标志数组

  std::shared_ptr<PotentialCalculator> p_calc_ =
      nullptr;  ///< 计算地图势场值的类

  int currentEnd_ = 0;  ///< 数组的末位索引
  int nextEnd_ = 0;
  int overEnd_ = 0;

  bool precise_ = false;
  bool use_distance_map_ = false;  ///< 势场计算是否用距离值

  float threshold_ = 0.0;          ///< 当前阈值
  float priorityIncrement_ = 0.0;  ///< 优先级阈值增量
};
}  // namespace CVTE_BABOT

#endif  // end of __D_STAR_HPP