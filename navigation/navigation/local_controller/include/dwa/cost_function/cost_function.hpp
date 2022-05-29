/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file cost_function.hpp
 *
 *@brief DWA评分函数的基类
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-09
 ************************************************************************/

#ifndef __COST_FUNCTION_HPP
#define __COST_FUNCTION_HPP
#include <string>

namespace CVTE_BABOT {

class Trajectory;

/**
 * CostFunction
 * @brief 评分函数的基类
 **/
class CostFunction {
 public:
  CostFunction() : scale_(1.0){};
  CostFunction(const CostFunction &obj) { scale_ = obj.scale_; }
  CostFunction &operator=(const CostFunction &obj) {
    if (this == &obj) {
      return *this;
    }
    scale_ = obj.scale_;
    return *this;
  }
  virtual ~CostFunction() = default;

  /**
   *setName
   *@brief
   *简介
   *
   *@param[in] s_name-函数名字
  **/
  void setName(const std::string &s_name) { s_name_ = s_name; }

  /**
   *getName
   *@brief
   *获取函数名字
   *
   *@return 函数名字
  **/
  std::string getName() { return s_name_; }

  /**
* prepare
* @brief
*   用来检查各个代价函数是否准备完成
*
* */
  virtual bool prepare() { return true; };

  /**
* scoreTrajectory
* @brief
*   计算一条路径的得分
*
* @param[in] traj-需要计算分数的路径
* */
  virtual double scoreTrajectory(const Trajectory &traj) = 0;

  /**
* setScale
* @brief
*   设置权重值
*
* @param[in] scale-权重值
* */
  inline void setScale(const double &scale) { scale_ = scale; }

  /**
* getScale
* @brief
*   获取权重值
* */
  inline double getScale() { return scale_; }

 protected:
  CostFunction(double scale = 1.0) : scale_(scale) {}

 private:
  double scale_;        ///< 比例
  std::string s_name_;  ///< 函数名
};

}  // namespace CVTE_BABOT

#endif  // __TRAJECTORTY_COST_FUNCTION_HPP