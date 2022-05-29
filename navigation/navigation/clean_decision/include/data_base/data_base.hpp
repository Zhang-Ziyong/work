/*
 * @Author: your name
 * @Date: 2021-09-28 18:15:23
 * @LastEditTime: 2021-10-21 16:01:57
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: /navigation/navigation/clean_decision/data_base/data_base.hpp
 */
#ifndef DATA_BASE_HPP_
#include "eigen3/Eigen/Core"
#include <vector>
namespace CVTE_BABOT {
typedef Eigen::Vector2d Vectex;
typedef std::vector<Vectex> Area;
typedef std::vector<Area> MutilArea;
}  // namespace CVTE_BABOT

#define DATA_BASE_HPP_
#endif