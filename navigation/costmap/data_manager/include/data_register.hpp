/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file data_register.hpp
 *
 *@brief hpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.3.0
 *@data 2020-09-01
 ************************************************************************/
#ifndef __DATA_REGISTER_HPP
#define __DATA_REGISTER_HPP

namespace CVTE_BABOT {
class DataRegister {
 public:
  DataRegister() = default;
  ~DataRegister() = default;

  DataRegister(const DataRegister &) = delete;
  DataRegister &operator=(const DataRegister &) = delete;

  static void createSubscribers();
};
}  // namespace CVTE_BABOT
#endif
