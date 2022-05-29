/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file classifier.hpp
 *
 *@brief hpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.8
 *@data 2020-07-22
 ************************************************************************/
#ifndef __CLASSIFIER_HPP
#define __CLASSIFIER_HPP
#include "common/tracking_utils.hpp"

namespace CVTE_BABOT {
struct ClassifierParams {
  double max_length = 0.00;
  double max_width = 0.00;
  double max_area = 0.00;
  double max_velocity = 0.00;
  double min_velocity = 0.00;
  double low_obs_height_threshhold = 0.00;
};

class Classifier {
 public:
  Classifier() = default;
  ~Classifier() = default;

  Classifier(const Classifier &) = delete;
  Classifier &operator=(const Classifier &) = delete;

  void updateParams(const ClassifierParams &classifier_params) {
    classifier_params_ = classifier_params;
  }

  void classify(const std::vector<ObjectPtr> &objects_tracked,
                std::vector<ConstObjectPtr> &objects_classified);

 private:
  /**
   *ifObjSizeOdd
   *@brief
   *判断对象的大小是否是否超出正常范围,如果不是,处理时认为是静态障碍
   *
   *@param[in] in-对象
   *@return true-是, false-否
  **/
  bool ifObjSizeOdd(const ConstObjectPtr &ptr_object);

  bool ifObjLow(const ConstObjectPtr &ptr_object);

  /**
   *ifObjstatic
   *@brief
   *判断对象是否静止的(新对象认为是动态的,因为没有办法计算速度)
   *
   *@param[in] in-对象
   *@return true-是, false-否
  **/
  bool ifObjStatic(const ConstObjectPtr &ptr_object);

  bool ifObjMobileCar(const ConstObjectPtr &ptr_object);

  ClassifierParams classifier_params_;
};
}  // namespace CVTE_BABOT
#endif
