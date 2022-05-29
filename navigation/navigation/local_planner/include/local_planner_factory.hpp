/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file local_planner_factory.hpp
 *
 *@brief 局部路径规划的工厂类
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version Navigation-v2.0
 *@data 2019-12-17
 ************************************************************************/

#ifndef __LOCAL_PLANNER_FACTORY_HPP
#define __LOCAL_PLANNER_FACTORY_HPP

#include <memory>
#include <string>

namespace CVTE_BABOT {

class LocalPlanner;

class LCPlannerFactory{
    friend class LCPlannerFactoryTest;
public:
    LCPlannerFactory() = default;
    LCPlannerFactory(const LCPlannerFactory &obj) = delete;
    LCPlannerFactory &operator=(const LCPlannerFactory &obj) = delete;
    ~LCPlannerFactory() = default;

    /**
     * createGlobalPlanner
     * @brief
     *   创建局部路径规划类的工厂函数
     *
     * @param[in] planner_name-需要创建的规划算法，目前支持 dijkstra和hybrid两种
     * @param[in] map_width-规划路径地图的宽度
     * @param[in] map_height-规划路径地图的高度
     * @param[out] 不同规划算法的实例指针
     * */
    std::shared_ptr<LocalPlanner> createLocalPlanner(const std::string &planner_name, 
        const unsigned int &map_width, const unsigned int &map_height);

private:

};

} // namcespace CVTE_BABOT

#endif // __LOCAL_PLANNER_FACTORY_HPP
