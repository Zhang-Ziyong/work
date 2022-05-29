/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file local_planner_factory.hpp
 *
 *@brief 局部路径规划工厂类生产方式实现
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version Navigation-v2.0
 *@data 2019-12-17
 ************************************************************************/

#include "local_planner_factory.hpp"
#include "local_planner.hpp"

#include <glog/logging.h>

namespace CVTE_BABOT {

std::shared_ptr<LocalPlanner> LCPlannerFactory::createLocalPlanner(const std::string &planner_name, 
        const unsigned int &map_width, const unsigned int &map_height) {
    if (planner_name == "dijkstra") {
        LOG(INFO) << "Use Dijkstra As Local Planner.";
        return std::make_shared<DijkstraLocalPlanner>(map_width, map_height);
    } else if (planner_name == "hybrid") {
        LOG(INFO) << "Use Hybrid As Local Planner.";
        return std::make_shared<HybridLocalPlanner>(map_width, map_height);
    } else if (planner_name == "kinodynamic") {
        LOG(INFO) << "Use kinodynamic As Local Planner.";
        return std::make_shared<KinodynamicLocalPlanner>();
    } else {
        LOG(INFO) << "Can't use " << planner_name << " as Local Planner."
            << " Default Dijkstra will be setted.";
        return std::make_shared<DijkstraLocalPlanner>(map_width, map_height);
    }
}

} // end of CVTE_BABOT

