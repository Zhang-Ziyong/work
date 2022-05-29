/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file mean_filter.cpp
 *
 *@brief cpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.4
 *@data 2020-05-20
 ************************************************************************/
#include "filters/mean_filter.hpp"
#include "assert.h"

namespace CVTE_BABOT {

void MeanFilter::filter(const double &origin_x, const double &origin_y,
                        double &filtered_x, double &filtered_y) {
  assert(de_x_.size() == de_y_.size());
  if (de_x_.size() < filter_size_) {
    de_x_.push_back(origin_x);
    de_y_.push_back(origin_y);
    filtered_x = de_x_.back();
    filtered_y = de_y_.back();
  } else {
    de_x_.push_back(origin_x);
    de_y_.push_back(origin_y);
    de_x_.pop_front();
    de_y_.pop_front();

    double sum_x = 0.0, sum_y = 0.0;
    for (size_t i = 0; i < de_y_.size(); i++) {
      sum_x += de_x_[i];
      sum_y += de_y_[i];
    }

    filtered_x = sum_x / de_x_.size();
    filtered_y = sum_y / de_y_.size();
  }
}

}  // namespace CVTE_BABOT
