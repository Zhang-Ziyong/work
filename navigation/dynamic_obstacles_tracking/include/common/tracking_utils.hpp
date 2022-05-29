/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file tracking_utils.hpp
 *
 *@brief hpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-12-05
 ************************************************************************/
#ifndef TRACKING_UTILS_HPP_
#define TRACKING_UTILS_HPP_
#include "object.hpp"

namespace CVTE_BABOT {
const int MAX_LABELS_NUM = 100;

const float EPSILON = 1e-9;

inline bool isElementInVector(std::vector<int> v, int element) {
  auto it = find(v.begin(), v.end(), element);
  if (it != v.end()) {
    return true;
  } else {
    return false;
  }
}

inline bool areaCmp(const ConstObjectPtr &obj1, const ConstObjectPtr &obj2) {
  double area1 = obj1->length * obj1->width;
  double area2 = obj2->length * obj2->width;
  return area1 > area2;
}
}  // namespace CVTE_BABOT

#endif
