/*
 * @Author: linyanlong
 * @Date: 2021-08-19 11:35:07
 * @LastEditTime: 2021-08-20 12:23:43
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath:
 * /navigation/navigation/planner_decision/config/planner_decision_config.hpp
 */
#ifndef PLANNER_DECISION_CONFIG_HPP_
#define PLANNER_DECISION_CONFIG_HPP_
#include <vector>
#include "eigen3/Eigen/Core"
struct PlannerDecisionConfig {
  PlannerDecisionConfig() {
    foot_print.push_back(Eigen::Vector2d(0.5, 0.35));
    foot_print.push_back(Eigen::Vector2d(-0.2, -0.35));
    dangerous_value = 100;
    bussiness_dist = 1.5;
    dynamic_dist = 2.0;
    bussiness_range = 0.5;
    k_safe = 0.5;
    k_bussiness = 0.5;
    k_dynamic = 0.5;
    k_mission = 0.4;
    map_range = 7.0;
    step = 0.3;
    check_map_range = false;
    min_cost = 120;
    max_cost = 170;
  }
  PlannerDecisionConfig(const std::vector<Eigen::Vector2d> &_foot_print,
                        int _dangerous_value, double _dynamic_dist,
                        double _bussiness_range, double _map_range,
                        double _k_safe, double _k_bussiness, double _k_dynamic,
                        double _k_mission) {
    foot_print = _foot_print;
    dangerous_value = _dangerous_value;
    dynamic_dist = _dynamic_dist;
    bussiness_range = _bussiness_range;
    map_range = _map_range;
    k_safe = _k_safe;
    k_bussiness = _k_bussiness;
    k_dynamic = _k_dynamic;
    k_mission = _k_mission;
  }
  std::vector<Eigen::Vector2d> foot_print;
  int dangerous_value;
  double bussiness_dist;
  double dynamic_dist;
  double bussiness_range;
  double map_range;
  double k_safe;
  double k_bussiness;
  double k_dynamic;
  double k_mission;
  double step = 0.1;
  bool check_map_range;
  int min_cost;
  int max_cost;
};

#endif
