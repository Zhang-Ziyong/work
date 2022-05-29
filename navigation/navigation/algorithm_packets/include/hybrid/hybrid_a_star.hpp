/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file hybrid_a_star.hpp
 *
 *@brief hybrid a* 算法的头文件函数
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version Navigation-v2.0
 *@data 2019-12-17
 ************************************************************************/

#ifndef __HYBRID_A_STAR_HPP
#define __HYBRID_A_STAR_HPP

#include "abstract_planner.hpp"
#include "lookup.hpp"
#include "smoother.hpp"
#include "collisiondetection.hpp"

#include <boost/smart_ptr.hpp>

namespace CVTE_BABOT {

/**
 * HybridAStartPlanner
 * @brief
 *   hybrid a*算法的实现接口
 * */

class HybridAStartPlanner/* : public AbstractPlannerBase*/ {
public:
  HybridAStartPlanner(const int &width, const int &height);
  HybridAStartPlanner() = delete;
  ~HybridAStartPlanner() = default;

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

private:
  void initializeLookups();
  bool setMap(boost::shared_array<unsigned char> &costs);

  //   Path path;
  Smoother smoother;
  //   Path smoothedPath = Path(true);
  DynamicVoronoi voronoiDiagram;
  CollisionDetection configurationSpace;

  Constants::config collisionLookup[Constants::headings * Constants::positions];

  float *dubinsLookup =
      new float[Constants::headings * Constants::headings *
                Constants::dubinsWidth * Constants::dubinsWidth];

  int width_ = 0;
  int height_ = 0;
};
} // namespace CVTE_BABOT

#endif // end of __HYBRID_A_STAR_HPP