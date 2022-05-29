/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file potential_calculator.cpp
 *
 *@brief 地图势场的计算方法的实现
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version Navigation-v2.0
 *@data 2019-12-17
 ************************************************************************/

#include "dijkstra/potential_calculator.hpp"
#include <math.h>
#include <iostream>

namespace CVTE_BABOT {

PotentialCalculator::PotentialCalculator(const int &nx, const int &ny) {
  setSize(nx, ny);
}

void PotentialCalculator::setSize(const int &nx, const int &ny) {
  nx_ = nx;
  ny_ = ny;
  ns_ = nx * ny;
}

float PotentialCalculator::calculatePotential(
    boost::shared_array<float> potential, const float &cost,
    const int &n) {
  // n 点在一维势场地图 上、下、左、右四个方向的势场值
  float l = potential[n - 1];
  float r = potential[n + 1];
  float u = potential[n - nx_];
  float d = potential[n + nx_];
  // 寻找四个方向势场最小的栅格点
  float tc = std::min(l, r);
  float ta = std::min(u, d);
  float hf = cost;
  float dc = std::fabs(tc - ta);
  ta = std::min(tc, ta);

  // 通过一个二次函数计算点的代价值，会比单纯相加的方法会更平滑
  // 详细介绍可参考：https://answers.ros.org/question/11388/navfn-algorism/?answer=16891#answer-container-16891
  if (dc >= hf) {
    return ta + hf;
  } else {
    float d = dc / hf;
    float v = -0.2301 * d * d + 0.5307 * d + 0.704;
    return ta + hf * v;
  }
}

}