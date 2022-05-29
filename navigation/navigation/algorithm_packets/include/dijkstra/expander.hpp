/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file expander.hpp
 *
 *@brief 可用于A星或D星规划算法的基类
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version Navigation-v2.0
 *@data 2019-12-16
 ************************************************************************/

#ifndef __EXPANDER_HPP
#define __EXPANDER_HPP

#include <boost/smart_ptr.hpp>
#include <memory>
#include <pose2d/pose2d.hpp>
#include <vector>

namespace CVTE_BABOT {
/**
 * Expander
 * @brief
 *   搜索算法的基类
 * */
const double POT_HIGH = 1.0e10;  ///< unassigned cell potential
class Expander {
 public:
  Expander() = default;
  Expander(const Expander &obj) = delete;
  Expander &operator=(const Expander &obj) = delete;
  virtual ~Expander() = default;

  /**
   * Expander
   * @brief
   *   构造函数，设置势场的地图大小
   *
   * @param[in] nx-地图x轴长度
   * @param[in] ny-地图y轴长度
   * */
  Expander(const unsigned int &nx, const unsigned int &ny);

  /**
   * setLethalCost
   * @brief
   *   设置致命代价值
   *
   * @param[in] lethal_cost-致命的代价值
   *
   * */
  inline void setLethalCost(const unsigned char &lethal_cost) {
    lethal_cost_ = lethal_cost;
  }

  /**
   * setFactor
   * @brief
   *   设置因子值
   *
   * @param[in] factor-因子值大小
   *
   * */
  inline void setFactor(const float &factor) { factor_ = factor; }

  /**
   * setHasUnknown
   * @brief
   *   设置是否存在未知区域
   *
   * @param[in] unknown-true/false 代表存在或不存在未知区域
   *
   * */
  inline void setHasUnknown(const bool &unknown) { unknown_ = unknown; }

  /**
   * getPath
   * @brief
   *   从计算得到的势场地图中查找路径
   *
   * @param[in] start_x-起点的x坐标值
   * @param[in] start_y-起点的y坐标值
   * @param[in] end_x-终点的x坐标值
   * @param[in] end_y-终点的y坐标值
   * @param[out] path-路径结果
   *
   * @return true - 代表搜索到路径 ，否则相反
   * */
  bool getPath(const double &start_x, const double &start_y,
               const double &end_x, const double &end_y,
               std::vector<Pose2d> &path);

  /**
   * toIndex
   * @brief
   *   二维坐标系到一维数组的索引转换
   *
   * @param[in] x-坐标点的x值
   * @param[in] y-坐标点的y值
   * */
  inline int toIndex(const int &x, const int &y) { return x + nx_ * y; }

  /**
   * setPreciseStart
   * @brief
   *   设置是否使用精确值更新势场值
   *
   * @param[in] precise-true/false 代表使用或不使用精确值计算
   *
   * */
  virtual void setPreciseStart(const bool &precise) { precise_ = precise; }

  /**
   * setNeutralCost
   * @brief
   *   设置中立的代价值
   *
   * @param[in] neutral_cost-中立的代价值大小
   *
   * */
  virtual void setNeutralCost(const unsigned char &neutral_cost) {
    neutral_cost_ = neutral_cost;
  }

  /**
   * setSize
   * @brief
   *   重新设置势场的地图大小
   *
   * @param[in] nx-地图x轴长度
   * @param[in] ny-地图y轴长度
   * */
  virtual void setSize(const unsigned int &nx, const unsigned int &ny);

  /**
   * gradCell
   * @brief
   *   计算一个点的梯度
   *
   * @param[in] n-计算点的索引值
   * @return 返回梯度大小
   * */
  float gradCell(const unsigned int &n);

  /**
   * getPotentialMap
   * @brief
   *   获取势场地图的API
   *
   * @return 直线势场地图的指针
   * */
  boost::shared_array<float> getPotentialMap() { return potential_; }

  /**
   * calculatePotentials
   * @brief
   *   根据起点和目标点更新势场值大小
   *
   * @param[in] costs-代价地图
   * @param[in] start_x-起点的x坐标值
   * @param[in] start_y-起点的y坐标值
   * @param[in] end_x-终点的x坐标值
   * @param[in] end_y-终点的y坐标值
   * @param[in] cycles-循环次数
   *
   * @return true - 代表搜索到路径 ，否则相反
   * */
  virtual bool calculatePotentials(
      const boost::shared_array<unsigned char> &costs, const double &start_x,
      const double &start_y, const double &end_x, const double &end_y,
      const int &cycles) = 0;

 protected:
  unsigned int nx_ = 0;    ///< 地图的长（单位：像素）
  unsigned int ny_ = 0;    ///< 地图的宽（单位：像素）
  unsigned int ns_ = 0;    ///< 地图的总大小（单位：像素）
  int cells_visited_ = 0;  ///< 用来统计已访问点数量
  float factor_ = 0.8;     ///< 比例因子
  float pathStep_ = 0.5;   ///< 梯度路径步长
  float map_resolution_ =
      0.1;  ///< 用于计算距离势场的地图分辨率，与costmap保持一致（单位：米）
  bool unknown_ = false;             ///< 搜索未知区域
  bool precise_ = true;              ///<
  unsigned char lethal_cost_ = 254;  ///< 致命的代价值
  unsigned char neutral_cost_ = 50;  ///< 中性的代价值

  boost::shared_array<float> potential_ = nullptr;  ///< 梯度数组
  boost::shared_array<float> gradx_ = nullptr;  ///< 梯度数组 x方向大小
  boost::shared_array<float> grady_ = nullptr;  ///< 梯度数组 y方向大小
};

}  // namespace CVTE_BABOT

#endif  // enf of __EXPANDER_HPPf