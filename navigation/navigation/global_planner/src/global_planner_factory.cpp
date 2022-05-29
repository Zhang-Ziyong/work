/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file global_planner_factory.hpp
 *
 *@brief 全局路径规划工厂类生产方式实现
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version Navigation-v2.0
 *@data 2019-12-17
 ************************************************************************/

#include "global_planner_factory.hpp"
#include "global_planner.hpp"

#include "dijkstra/D_star.hpp"
#include "hybrid/hybrid_a_star.hpp"
#include "kinodynamic/kinodynamic_astar.hpp"

#include <glog/logging.h>

namespace CVTE_BABOT {

std::shared_ptr<GlobalPlanner> GBPlannerFactory::createGlobalPlanner(const std::string &planner_name, 
        const unsigned int &map_width, const unsigned int &map_height) {
    if (planner_name == "dijkstra") {
        LOG(INFO) << "Use Dijkstra As Global Planner.";
        return std::make_shared<DijkstraGlobalPlanner>(map_width, map_height);
    } else if (planner_name == "hybrid") {
        LOG(INFO) << "Use Hybrid As Global Planner.";
        return std::make_shared<HybridGlobalPlanner>(map_width, map_height);
    } else if (planner_name == "kinodynamic") {
        LOG(INFO) << "Use kinodynamic As Global Planner.";
        return std::make_shared<KinodynamicGlobalPlanner>();
    } else {
        LOG(INFO) << "Can't use " << planner_name << " as Global Planner."
            << " Default Dijkstra will be setted.";
        return std::make_shared<DijkstraGlobalPlanner>(map_width, map_height);
    }
}

} // end of CVTE_BABOT

