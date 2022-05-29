/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file debug_log.hpp
 *
 *@brief 可以设置是否打印log及level.
 *
 *@modified by wuhuabo(wuhuabo@cvte.com)
 *
 *@author wuhuabo(wuhuabo@cvte.com)
 *@version current_algo.dev.2.0
 *@data 2019-06-28
 ************************************************************************/
#ifndef __DEBUG_LOG_HPP
#define __DEBUG_LOG_HPP

#include "glog/logging.h"
using google::WARNING;
using google::INFO;
using google::ERROR;

/*
 *DLOG
 *@brief
 *封装glog，用于开关打印
 *
*/
class DLog {
 public:
  DLog() = default;
  ~DLog() = default;

  DLog(const DLog&) = delete;
  DLog& operator()(DLog&) = delete;

  /**
   *setDebugFlag
   *@brief
   *设置是否打印log,在初始化时设置一次即可，在多线程中同时修改与读取会出错
   *
   *@param[in] print_log-是否打印log
   * */
  void setDebugFlag(const bool& print_log) { b_print_log_ = print_log; }

  /**
   *operator()
   *@brief
   *重载（），用于打印log
   *
   *@param[in] level-打印级别，与glog一致，WARNING, INFO,ERROR三个级别
   *@param[in] log-log内容
   * */
  void operator()(const int& level, const std::string& log) {
    if (!b_print_log_) {
      return;
    }
    switch (level) {
      case google::WARNING:
        LOG(WARNING) << log;
        break;
      case google::INFO:
        LOG(INFO) << log;
        break;
      case google::ERROR:
        LOG(ERROR) << log;
        break;
      default:
        LOG(INFO) << log;
        break;
    }
  }

 private:
  bool b_print_log_ = false;  ///< 是否打印log
};

extern DLog DELOG;  ///<全局变量，所有包含此头文件的共享
// 在头文件中用static定义的话起不到这个效果，会变成每个包含此头文件的声明了一个当前文件的全局静态变量

/**
 *ENABLE_DEBUG_LOG
 *@brief
 *打印log
 * */
#define ENABLE_DEBUG_LOG DELOG.setDebugFlag(true);

/**
 *DISABLE_DEBUG_LOG
 *@brief
 *不打印log
 * */
#define DISABLE_DEBUG_LOG DELOG.setDebugFlag(false);

#endif
