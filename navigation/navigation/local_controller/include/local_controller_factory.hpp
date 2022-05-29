/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file local_controller_factory.hpp
 *
 *@brief 局部路径规划控制器的生产工厂
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version Navigation-v2.0
 *@data 2019-12-25
 ************************************************************************/

#ifndef __LOCAL_CONTROLLER_FACTORY_HPP
#define __LOCAL_CONTROLLER_FACTORY_HPP

#include <glog/logging.h>
#include <memory>
#include <string>
#include "dwa/dwa_planner.hpp"
#include "lypu/lypu_planner.hpp"
#include "stanley/stanley_controller.hpp"
#include "pid/pid_planner.hpp"
#include "edge/edge_controller.hpp"
// #include "mpc/mpc_controller.hpp"
namespace CVTE_BABOT {

class LocalController;
class Costmap2d;

class LocalControllerFactory {
 public:
  LocalControllerFactory() = default;
  LocalControllerFactory(const LocalControllerFactory &obj) = delete;
  LocalControllerFactory &operator=(const LocalControllerFactory &obj) = delete;
  ~LocalControllerFactory() = default;

  /**
   * createLocalController
   * @brief
   *   创建全局路径规划类的工厂函数
   *
   * @param[in] controller_name-需要创建控制算法，目前只支持dwa
   * @param[in] costmap-costmap指针
   * @param[out] 不同控制算法的实例指针
   * */
  std::shared_ptr<LocalController> createLocalController(
      const std::string &controller_name, std::shared_ptr<Costmap2d> costmap) {
    if (controller_name == "dwa") {
      LOG(INFO) << "Use DWA As LocalController.";
      return std::make_shared<DWALocalController>(costmap);
      // } else if (controller_name == "pid") {
      //   LOG(INFO) << "Use PID As LocalController.";
      //   return std::make_shared<PIDLocalController>(costmap);
    } else if (controller_name == "lypu") {
      LOG(INFO) << "Use LYPU As LocalController.";
      return std::make_shared<LYPULocalController>();
    } else if (controller_name == "mpc") {
      // return std::make_shared<MPCLocalController>();
    } else if (controller_name == "stanley") {
      LOG(INFO) << "Use stanley As LocalController";
      return std::make_shared<StanleyController>();
    } else if (controller_name == "edge") {
      LOG(INFO) << "Use edge As LocalController";
      return std::make_shared<EdgeController>();
    } else {
      LOG(INFO) << "Can't use " << controller_name << " as Local Controller."
                << " Default DWA will be setted.";
      return std::make_shared<DWALocalController>(costmap);
    }
  }
};

}  // namespace CVTE_BABOT

#endif  // __LOCAL_CONTROLLER_FACTORY_HPP
