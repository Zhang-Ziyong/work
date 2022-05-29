/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file bounding.hpp
 *
 *@brief hpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.3.0
 *@data 2020-11-26
 ************************************************************************/
#ifndef __BOUNDING_HPP
#define __BOUNDING_HPP
#include <vector>
#include <algorithm>

#include "tracking_utils.hpp"
namespace CVTE_BABOT {
/*
*                           |x
*(x_max,y_max) ---width-----|
*      |                    |
*      |                  length
*      |                    |
*  y<-----------------(x_min,y_min)
*/
typedef struct {
  double x_min;  // right-bottom corner
  double y_min;
  double x_max;  // left-top corner
  double y_max;
} GroundBox;

/**
 * @brief Object's 3D OBB to 2D ground box
 * @param object
 * @param gbox
 */
void toGroundBox(ConstObjectPtr object, GroundBox *gbox) {
  gbox->x_min = object->object_center(0) - object->length / 2;
  gbox->y_min = object->object_center(1) - object->width / 2;
  gbox->x_max = object->object_center(0) + object->length / 2;
  gbox->y_max = object->object_center(1) + object->width / 2;
  // LOG(INFO) << "origin: " << object->object_center(0) << ", "
  //           << object->object_center(1);
  // LOG(INFO) << "x: " << gbox->x_min << ", " << gbox->x_max;
  // LOG(INFO) << "y: " << gbox->y_min << ", " << gbox->y_max;
}

bool aInsideB(const GroundBox &box1, const GroundBox &box2) {
  if (box2.x_min <= box1.x_min && box2.y_min <= box1.y_min &&
      box1.x_max <= box2.x_max && box1.y_max <= box2.y_max) {
    return true;
  } else {
    return false;
  }
}

/**
 * @brief Intersection-over-Union
 */
double groundBoxIoU(const GroundBox &box1, const GroundBox &box2) {
  double start_x = std::min(box1.x_min, box2.x_min);
  double end_x = std::max(box1.x_max, box2.x_max);
  double length1 = box1.x_max - box1.x_min;
  double length2 = box2.x_max - box2.x_min;
  double length = length1 + length2 - (end_x - start_x);
  // LOG(INFO) << "1: " << start_x << ", " << end_x;
  // LOG(INFO) << "1: " << length1 << ", " << length2;

  double start_y = std::min(box1.y_min, box2.y_min);
  double end_y = std::max(box1.y_max, box2.y_max);
  double width1 = box1.y_max - box1.y_min;
  double width2 = box2.y_max - box2.y_min;
  double width = width1 + width2 - (end_y - start_y);
  // LOG(INFO) << "2: " << start_y << ", " << end_y;
  // LOG(INFO) << "2: " << width1 << ", " << width2;

  if (width <= 0 || length <= 0) {
    return 0.00;  // 重叠率为0
  } else {
    //两矩形相交面积
    double in = length * width;

    // LOG(INFO) << "in: " << in << ", " << length << ", " << width;
    // 返回最大值，因为目的不是匹配而是评判相交度
    double area1 = length1 * width1;
    double area2 = length2 * width2;
    double ratio1 = in / area1;
    double ratio2 = in / area2;
    return std::max(ratio1, ratio2);
  }
}

/**
 * @brief check box1 is overlapping with box2
 *  true: box1 is inside box2 or box2 is inside box1
 *  true: IoU between box1 and box2 > threshold_IoU
 * @param box1
 * @param box2
 * @param threshold_IoU
 * @return
 */
bool groundBoxOverlap(const GroundBox &box1, const GroundBox &box2,
                      double threshold_IoU) {
  if (aInsideB(box1, box2) || aInsideB(box2, box1)) {
    return true;
  }

  if (groundBoxIoU(box1, box2) > threshold_IoU) {
    return true;
  }

  return false;
}

}  // namespace CVTE_BABOT
#endif
