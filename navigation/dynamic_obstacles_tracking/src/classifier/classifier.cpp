/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file classifier.cpp
 *
 *@brief cpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.8
 *@data 2020-07-22
 ************************************************************************/
#include "classifier/classifier.hpp"
#include <glog/logging.h>

namespace CVTE_BABOT {
void Classifier::classify(const std::vector<ObjectPtr> &objects_tracked,
                          std::vector<ConstObjectPtr> &objects_classified) {
  objects_classified.clear();
  objects_classified.reserve(objects_tracked.size());
  for (const auto &ptr_object : objects_tracked) {
    objects_classified.push_back(ptr_object);
    // 高度小于阀值当作静态障碍处理
    // 把最大高度低于一定高度的障碍认为是静态障碍,因为比较矮的障碍,
    // 一般不是运动的障碍,而且需要避免在盲区
    // 内时被清除掉,需要标记成静态障碍用3d方式处理.
    if (ifObjLow(ptr_object)) {
      ptr_object->object_type = STATIC_OBJECT;
    } else if (ifObjMobileCar(ptr_object)) {
      // 标记类型,目前不是汽车的话标为OTHER_OBJECT
      ptr_object->object_type = CAR_OBJECT;
    } else {
      ptr_object->object_type = OTHER_OBJECT;
    }
  }
}

bool Classifier::ifObjSizeOdd(const ConstObjectPtr &ptr_object) {
  return (ptr_object->length * ptr_object->width >
              classifier_params_.max_area ||
          ptr_object->length > classifier_params_.max_length ||
          ptr_object->width > classifier_params_.max_width);
}

bool Classifier::ifObjLow(const ConstObjectPtr &ptr_object) {
  return ptr_object->object_center(2) <
         classifier_params_.low_obs_height_threshhold;
}

bool Classifier::ifObjStatic(const ConstObjectPtr &ptr_object) {
  // 新对象跟踪时没有速度,认为是动态的.
  if (ptr_object->tracking_state == NEW_OBJECT) {
    return false;
  }

  double velocity_x = ptr_object->velocity(0);
  double velocity_y = ptr_object->velocity(1);
  if (ptr_object->motion_state == IMMOVABLE_OBJECT &&
      velocity_x * velocity_x + velocity_y * velocity_y <
          classifier_params_.min_velocity * classifier_params_.min_velocity) {
    return true;
  } else {
    return false;
  }
}

bool Classifier::ifObjMobileCar(const ConstObjectPtr &ptr_object) {
  // 如果先前为汽车，则不改变其类型
  return (ptr_object->object_type == CAR_OBJECT) ||
         (ptr_object->motion_state == MOBILE_OBJECT &&
          ptr_object->height > 0.6 &&
          (ptr_object->length * ptr_object->width > 1.5 ||
           ptr_object->length > 1.0 || ptr_object->width > 1.0));
}

}  // namespace CVTE_BABOT
