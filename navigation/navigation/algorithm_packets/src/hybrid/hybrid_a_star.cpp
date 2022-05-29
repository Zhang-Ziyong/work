/************************************************************************
 * Software License Agreement (BSD License)
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file hybrid_a_star.cpp
 *
 *@brief hybrid_a_star算法的实现
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(liangchenmingjianjiajun@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-11
 ************************************************************************/

#include "hybrid/hybrid_a_star.hpp"
#include "hybrid/algorithm.hpp"

#include <glog/logging.h>
#include <iostream>
#include <tgmath.h>

namespace CVTE_BABOT {

HybridAStartPlanner::HybridAStartPlanner(const int &width, const int &height)
    : configurationSpace(width, height) {
  width_ = width;
  height_ = height;
  LOG(INFO) << "HybridAStartPlanner Constructor Finish";
}

void HybridAStartPlanner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }

  Lookup::collisionLookup(collisionLookup);
}

bool HybridAStartPlanner::setMap(boost::shared_array<unsigned char> &costs) {
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }

  configurationSpace.updateGrid(costs);

  bool **binMap;
  binMap = new bool *[width_];

  for (int x = 0; x < width_; x++) {
    binMap[x] = new bool[height_];
  }

  for (int x = 0; x < width_; ++x) {
    for (int y = 0; y < height_; ++y) {
      binMap[x][y] = costs[y * width_ + x] ? true : false;
    }
  }

  voronoiDiagram.initializeMap(width_, height_, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize();
  return true;
}

bool HybridAStartPlanner::makePlan(boost::shared_array<unsigned char> &costs,
                                   const Pose2d &start_pose,
                                   const Pose2d &end_pose,
                                   std::vector<Pose2d> &path_result) {

  setMap(costs);

  int depth = Constants::headings;
  int length = width_ * height_ * depth;
  // define list pointers and initialize lists
  Node3D *nodes3D = new Node3D[length]();
  Node2D *nodes2D = new Node2D[width_ * height_]();

  float t = Helper::normalizeHeadingRad(end_pose.yaw);
  const Node3D nGoal(end_pose.x, end_pose.y, t, 0, 0, nullptr);

  t = Helper::normalizeHeadingRad(start_pose.yaw);
  Node3D nStart(start_pose.x, start_pose.y, t, 0, 0, nullptr);

  // FIND THE PATH
  Node3D *nSolution =
      Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width_, height_,
                             configurationSpace, dubinsLookup);

  smoother.tracePath(nSolution);
  smoother.smoothPath(voronoiDiagram);

  auto node_path = smoother.getPath();
  Pose2d node_pose;
  for (size_t i = 0; i < node_path.size(); ++i) {
    node_pose.x = node_path[i].getX();
    node_pose.y = node_path[i].getY();
    node_pose.yaw = Helper::toRad(node_path[i].getT());
    path_result.push_back(node_pose);
  }

  return true;
}

} // namespace CVTE_BABOT