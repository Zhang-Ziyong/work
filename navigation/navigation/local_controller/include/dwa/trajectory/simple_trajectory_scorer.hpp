/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file simple_trajectory_scorer.hpp
 *
 *@brief 对采样轨迹进行打分
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-10
 ************************************************************************/

#ifndef __SIMPLE_TRAJECTORY_SCORER_HPP
#define __SIMPLE_TRAJECTORY_SCORER_HPP

#include <memory>
#include <vector>

namespace CVTE_BABOT {
class SimpleTrajectoryGenerator;
class CostFunction;
class Trajectory;

/**
* SimpleTrajectoryScorer
* @brief
*   对采样轨迹进行打分
* */

class SimpleTrajectoryScorer {
 public:
  SimpleTrajectoryScorer() = default;
  ~SimpleTrajectoryScorer() = default;
  
  /**
*SimpleTrajectoryScorer
*@brief
*  构造函数
*
*@param[in] gen_list-轨迹样本生成类
*@param[in] critics-评价函数集合
* */
  SimpleTrajectoryScorer(
      const std::shared_ptr<SimpleTrajectoryGenerator> gen_list,
      const std::vector<std::shared_ptr<CostFunction>> &critics);

  /**
*scoreTrajectory
*@brief
*  从各个评价函数对一条路径进行评分，如果找到负值评分或者比最优路径得分要大
*  则马上结束评分
*
*@param[in] traj-需要进行评分的轨迹
*@param[in] best_traj_cost-最优路径的得分
*@return 返回路径的总得分
* */
  double scoreTrajectory(const Trajectory &traj, const double &best_traj_cost);

  /**
*findBestTrajectory
*@brief
*  从路径集合中，寻找一条得分最优的路径
*
*@param[out] traj-最优路径的结果
*@param[in] all_explored-路径集合的指针
*@return true代表找到一条非负的最优路径
* */
  bool findBestTrajectory(
      Trajectory &traj, std::shared_ptr<std::vector<Trajectory>> all_explored);

 private:
  std::shared_ptr<SimpleTrajectoryGenerator> gen_list_ =
      nullptr;  ///< 轨迹样本生成类指针
  std::vector<std::shared_ptr<CostFunction>> critics_;  ///< 评价函数列表
};

}  // namespace CVTE_BABOT

#endif  // __SIMPLE_TRAJECTORY_SCORER_HPP