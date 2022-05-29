/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 201９, CVTE.
 * All rights reserved.
 *
 *@file tic_toc.hpp
 *
 *@brief
 * 时间测量类
 *
 *@author Wei Zhang(zhangwei@cvte.com)
 *@version 1.0
 *@data 2019-11-04
 ************************************************************************/
#ifndef TICTOC_H_
#define TICTOC_H_

#include <iostream>
#include <string>
#include <ctime>
#include <cstdlib>
#include <chrono>

namespace cvte_lidar_slam {

class TicToc {
 public:
  /**
   *TicToc
   *@brief
   *计时类
   *
   **/
  TicToc() { tic(); }

  /**
   *tic
   *@brief
   *当前时间
   *
   **/
  void tic() { start = std::chrono::system_clock::now(); }

  /**
   *toc
   *@brief
   *耗时计算
   *
   **/
  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000;
  }

  /**
   *print
   *@brief
   *耗时打印
   *param[in] info-要打印的信息
   *
   **/
  void print(const std::string &info) {
    std::cout << info + "  " << toc() << "  ms !!" << std::endl;
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
  ///< 起始和结束时间

};  // end of class

}  // namespace cvte_lidar_slam

#endif  // TICTOC_H_
