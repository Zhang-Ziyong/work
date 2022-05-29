/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file line_iterator.hpp
 *
 *@brief 布雷森漢姆直線演算法实现
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-10
 ************************************************************************/

#ifndef __LINE_ITERATOR_HPP
#define __LINE_ITERATOR_HPP

#include <stdlib.h>
namespace CVTE_BABOT {
/**
* LineIterator
* @brief
*   布雷森漢姆直線演算法实现
*   是用來描繪由兩點所決定的直線的演算法，它會算出一條線段在n維點陣圖上最接近的點
* */

class LineIterator {
 public:
  LineIterator() = delete;
  LineIterator(const LineIterator &obj) = delete;
  LineIterator &operator=(const LineIterator &obj) = delete;

  LineIterator(const int &x0, const int &y0, const int &x1, const int &y1)
      : x0_(x0),
        y0_(y0),
        x1_(x1),
        y1_(y1),
        x_(x0),
        y_(y0),
        deltax_(abs(x1 - x0)),
        deltay_(abs(y1 - y0)),
        curpixel_(0) {
    // X and Y start of at first endpoint.
    if (x1_ >= x0_) {
      // The x-values are increasing
      xinc1_ = 1;
      xinc2_ = 1;
    } else {
      // The x-values are decreasing
      xinc1_ = -1;
      xinc2_ = -1;
    }

    if (y1_ >= y0_) {
      // The y-values are increasing
      yinc1_ = 1;
      yinc2_ = 1;
    } else {
      // The y-values are decreasing
      yinc1_ = -1;
      yinc2_ = -1;
    }

    if (deltax_ >= deltay_) {
      // There is at least one x-value for every y-value
      xinc1_ = 0;
      // Don't change the x when numerator >= denominator
      yinc2_ = 0;
      // Don't change the y for every iteration
      den_ = deltax_;
      num_ = deltax_ / 2;
      numadd_ = deltay_;
      numpixels_ = deltax_;
      // There are more x-values than y-values
    } else {
      // There is at least one y-value for every x-value
      xinc2_ = 0;
      // Don't change the x for every iteration
      yinc1_ = 0;
      // Don't change the y when numerator >= denominator
      den_ = deltay_;
      num_ = deltay_ / 2;
      numadd_ = deltax_;
      numpixels_ = deltay_;
      // There are more y-values than x-values
    }
  }

  /**
* isValid
* @brief
*   判断直线是否非法越界
* */
  inline bool isValid() const { return curpixel_ <= numpixels_; }

  /**
* advance
* @brief
*   在直线两端选择栅格点
* */
  void advance() {
    num_ += numadd_;
    // Increase the numerator by the top of the fraction
    // Check if numerator >= denominator
    if (num_ >= den_) {
      num_ -= den_;
      // Calculate the new numerator value
      x_ += xinc1_;
      // Change the x as appropriate
      y_ += yinc1_;
      // Change the y as appropriate
    }
    x_ += xinc2_;
    // Change the x as appropriate
    y_ += yinc2_;
    // Change the y as appropriate

    curpixel_++;
  }
  /**
* getX
* @brief
*   获取直线起点的x坐标值
* */
  inline int getX() const { return x_; }

  /**
* getY
* @brief
*   获取直线起点的Y坐标值
* */
  inline int getY() const { return y_; }

  /**
* getX0
* @brief
*   获取直线终点的X坐标值
* */
  inline int getX0() const { return x0_; }

  /**
* getY0
* @brief
*   获取直线终点的Y坐标值
* */
  inline int getY0() const { return y0_; }

  /**
* getX1
* @brief
*   获取直线当前索引点的X坐标
* */
  inline int getX1() const { return x1_; }

  /**
* gegetY1tX1
* @brief
*   获取直线当前索引点的Y坐标
* */
  inline int getY1() const { return y1_; }

 private:
  int x0_ = 0;  ///< 直线起点的x值
  int y0_ = 0;  ///< 直线起点的y值
  int x1_ = 0;  ///< 直线终点的x值
  int y1_ = 0;  ///< 直线终点的y值
  int x_ = 0;   ///< 当前点的x值
  int y_ = 0;   ///< 当前点的x值

  int deltax_ = 0;  ///< x轴上的距离
  int deltay_ = 0;  ///< y轴上的距离;

  int curpixel_ = 0;  ///< 直线点中的索引变量

  int xinc1_ = 0;  ///< x轴反向的增量
  int xinc2_ = 0;  ///< x轴正向的增量
  int yinc1_ = 0;  ///< y轴反向的增量
  int yinc2_ = 0;  ///< y轴正向的增量

  ///< 这几个变量有待确定
  int den_ = 0;
  int num_ = 0;
  int numadd_ = 0;
  int numpixels_ = 0;
};
}  // namespace CVTE_BABOT

#endif  // __LINE_ITERATOR_HPP