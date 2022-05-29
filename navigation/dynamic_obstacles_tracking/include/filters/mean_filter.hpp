/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file mean_filter.hpp
 *
 *@brief hpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.4
 *@data 2020-05-20
 ************************************************************************/
#ifndef __MEAN_FILTER_HPP
#define __MEAN_FILTER_HPP
#include <algorithm>
#include <deque>

namespace CVTE_BABOT {
class MeanFilter {
 public:
  explicit MeanFilter(const unsigned int &filter_size)
      : filter_size_(filter_size) {}
  ~MeanFilter() = default;

  MeanFilter(const MeanFilter &) = default;
  MeanFilter &operator=(const MeanFilter &) = default;

  void reset() {
    de_x_.clear();
    de_y_.clear();
  }

  void filter(const double &origin_x, const double &origin_y,
              double &filtered_x, double &fitlered_y);

 private:
  double filter_size_ = 0.0;
  std::deque<double> de_x_;
  std::deque<double> de_y_;
};
}  // namespace CVTE_BABOT
#endif
