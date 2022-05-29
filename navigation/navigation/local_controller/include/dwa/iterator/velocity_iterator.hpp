/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file velocity_iterator.hpp
 *
 *@brief 用来存储最大最小速度之间的速度样本值
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-10
 ************************************************************************/

#ifndef __VELOCITY_ITERATOR_HPP
#define __VELOCITY_ITERATOR_HPP

#include <algorithm>
#include <cmath>

namespace CVTE_BABOT {
/**
* VelocityIterator
* @brief
*   用来存储最大最小速度之间的速度样本值
* */

class VelocityIterator {
 public:
  VelocityIterator() = delete;
  ~VelocityIterator() = default;
  VelocityIterator(const VelocityIterator &obj) = delete;
  VelocityIterator &operator=(const VelocityIterator &obj) = delete;

  VelocityIterator(const double &min, const double &max, int num_samples)
      : current_index_(0) {
    if (min == max) {
      samples_.push_back(min);
    } else {
      num_samples = std::max(2, num_samples);
      // e.g. for 4 samples, split distance in 3 even parts
      double step_size = (max - min) / double(std::max(1, (num_samples - 1)));

      // we make sure to avoid rounding errors around min and max.
      double current;
      double next = min;
      for (int j = 0; j < num_samples - 1; ++j) {
        current = next;
        next += step_size;
        samples_.push_back(current);
        // if 0 is among samples, this is never true. Else it inserts a 0
        // between the positive and negative samples
        // if ((current < 0) && (next > 0)) {
        //   samples_.push_back(0.0);
        // }
      }
      samples_.push_back(max);
    }
  }

  VelocityIterator &operator++(int) {
    current_index_++;
    return *this;
  }

  /**
* getVelocity
* @brief
*   获取速度样本
* */
  inline double getVelocity() { return samples_[current_index_]; }

  /**
* reset
* @brief
*  重置速度样本
* */
  void reset() { current_index_ = 0; }

  /**
* isFinished
* @brief
*   判断是否已经遍历完速度样本值
* */
  bool isFinished() { return current_index_ >= samples_.size(); }

 private:
  std::vector<double> samples_;  ///< 存储速度的样本值
  unsigned int current_index_;   ///< 记录的索引值
};

}  // namespace CVTE_BABOT

#endif  // __VELOCITY_ITERATOR_HPP