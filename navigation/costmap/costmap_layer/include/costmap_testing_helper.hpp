/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file costmap_testing_helper.hpp
 *
 *@brief 定义一些函数用于辅助单元测试
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified chenmingjian (chenmigjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-15
 ************************************************************************/
#ifndef __COSTMAP_TESTING_HELPER_HPP
#define __COSTMAP_TESTING_HELPER_HPP
#include <stdarg.h>
#include <iostream>
#include "costmap_2d.hpp"
#include "costmap_utils.hpp"
#include "inflation_layer.hpp"
#include "layered_costmap.hpp"
#include "obstacle_layer.hpp"
#include "range_sensor_layer.hpp"
#include "static_layer.hpp"
#include "voxel_layer.hpp"

namespace CVTE_BABOT {

// for test
template <class Value, class ExpectValue>
struct TestStruct {
  Value value;
  ExpectValue expect_value;
};

// in the variadic_arguments, two double group into a vertex, and there are
// four vertexs
void setArea(std::vector<CVTE_BABOT::WorldmapPoint> &area, ...) {
  area.clear();
  int n = 4;
  va_list args;
  va_start(args, area);
  while (n > 0) {
    area.push_back({va_arg(args, double), va_arg(args, double)});
    n--;
  }
  va_end(args);
}

char printableCost(unsigned char cost) {
  switch (cost) {
    case NO_INFORMATION:
      return '?';
    case LETHAL_OBSTACLE:
      return 'L';
    case INSCRIBED_INFLATED_OBSTACLE:
      return 'I';
    case FREE_SPACE:
      return '.';
    default:
      return '0' + (unsigned char)(10 * cost / 255);
  }
}

void printMap(Costmap2d &costmap) {
  printf("map:\n");
  for (unsigned int i = 0; i < costmap.getSizeInCellsY(); i++) {
    for (unsigned int j = 0; j < costmap.getSizeInCellsX(); j++) {
      // printf("%4d", int(costmap.getCost(j, i)));
      printf("%4c", printableCost(int(costmap.getCost(j, i))));
    }
    printf("\n\n");
  }
}

unsigned int countValues(Costmap2d &costmap, unsigned char value,
                         bool equal = true) {
  unsigned int count = 0;
  for (unsigned int i = 0; i < costmap.getSizeInCellsY(); i++) {
    for (unsigned int j = 0; j < costmap.getSizeInCellsX(); j++) {
      unsigned char c = costmap.getCost(j, i);
      if ((equal && c == value) || (!equal && c != value)) {
        count += 1;
      }
    }
  }
  return count;
}
}

#endif  // __COSTMAP_TESTING_HELPER_HPP
