/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022, CVTE.
 * All rights reserved.
 *
 *@file data_fliter.hpp
 *
 *@brief hpp模板
 *
 *@author chenweijian(chenweijian@cvte.com)
 *@version
 *@data 2022-05-11
 ************************************************************************/
#ifndef __DATA_FILTER_HPP
#define __DATA_FILTER_HPP
#include <map>
#include <vector>
#include <iostream>
#include "glog/logging.h"

namespace CVTE_BABOT {

template <class data_type>
class DataFilter {
 public:
  DataFilter() { iterator_ = data_.begin(); };
  DataFilter(size_t size) {
    creat(size);
    // filterFunIterator = this->filterFunction1;
  }
  ~DataFilter() = default;

  /**
   * @brief 创建空间
   *
   * @param size
   */
  void creat(size_t size) {
    data_.resize(size);
    iterator_ = data_.begin();
  }

  /**
   * @brief 推入参数
   *
   * @param value
   */
  void push(data_type value) {
    *iterator_ = value;

    if (++iterator_ == data_.end()) {
      iterator_ = data_.begin();
    }
  }

  data_type getFilterData() { return filterFunction1(); }

  data_type getLatestData() { return *iterator_; }

 private:
  std::vector<data_type> data_;  //
  typename std::vector<data_type>::iterator iterator_;
  // data_type (*filterFunIterator)();

  data_type filterFunction1() {
    data_type min = *data_.begin();
    data_type max = *data_.begin();
    data_type data_sum;
    for (auto iterator = data_.begin(); iterator != data_.end(); ++iterator) {
      if (*iterator < min) {
        min = *iterator;
      }
      if (*iterator > max) {
        max = *iterator;
      }
      data_sum += *iterator;
    }
    data_sum = data_sum - min - max;
    return (data_sum / (data_.size() - 2));
  };
};

}  // namespace CVTE_BABOT
#endif
