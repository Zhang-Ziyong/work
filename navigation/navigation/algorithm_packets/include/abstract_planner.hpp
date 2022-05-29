/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file abstract_planner.hpp
 *
 *@brief 规划算法的基类
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version Navigation-v2.0
 *@data 2019-12-16
 ************************************************************************/

#ifndef __ABSTRACT_PLANNER_HPP
#define __ABSTRACT_PLANNER_HPP

#include <pose2d/pose2d.hpp>
#include <boost/smart_ptr.hpp>
#include <memory>
#include <vector>

namespace CVTE_BABOT {
/**
 * Abstract
 * @brief
 *   规划算法的基类
 * */
class AbstractPlannerBase {
public:
  AbstractPlannerBase() = default;
  AbstractPlannerBase(const AbstractPlannerBase &obj) = delete;
  AbstractPlannerBase &operator=(const AbstractPlannerBase &obj) = delete;
  virtual ~AbstractPlannerBase() {};
    
  // 这个基类的作用不大，暂时先去除。后续有公用部分的时候，再抽出来。
  /**
   * makePlan
   * @brief
   *   输入一张地图，根据起点和终点规划出一条最短路径
   *   所有规划算法都应该提供这个规划的接口
   *
   * @param[in] costs-输入的代价地图
   * @param[in] start_x-起点x值
   * @param[in] start_y-起点y值
   * @param[in] end_x-终点x值
   * @param[in] end_y-终点y值
   * @param[out] n-二维坐标对应一维的索引点
   * @ return true-代表规划路径成功
   * */
//   virtual bool makePlan(boost::shared_array<unsigned char> &costs,
//                         const Pose2d &start_pose, const Pose2d &end_pose,
//                         std::vector<Pose2d> &path_result) = 0;
};

} // namespace CVTE_BABOT

#endif // enf of __ABSTRACT_PLANNER_HPP
