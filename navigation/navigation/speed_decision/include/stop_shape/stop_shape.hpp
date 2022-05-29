/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file stop_shape.hpp
 *
 *@brief 停障框描述
 *
 *@author chenweijian
 *@modified
 *@version
 *@data
 ************************************************************************/
#ifndef STOP_SHAPE_HPP_
#define STOP_SHAPE_HPP_
#include <array>
#include <vector>
#include <math.h>
#include <map>
#include "eigen3/Eigen/Core"
#include "speed_decision_config.hpp"

namespace CVTE_BABOT {
class StopPolygon {
 public:
  /**
   * @brief Construct a new Stop Polygon object
   *
   * @param[in] v_endpoint  端点
   * @param[in] start_theta 起始角度  default: 0
   * @param[in] end_theta   结束角度  default: 0
   */
  StopPolygon(const std::vector<Eigen::Vector2d> &v_endpoint,
              double start_theta = 0, double end_theta = 0);

  /**
   * @brief 检查点是否在多边形内
   *
   * @param[in] point 查询点
   * @return true
   * @return false
   */
  bool checkPonitInside(const Eigen::Vector2d &point);

  /**
   * @brief 设置偏移量
   *
   * @param[in] x_max_inc  x轴 正方向 增量
   * @param[in] x_min_inc  x轴 负方向 增量
   * @param[in] y_max_inc  y轴 正方向 增量
   * @param[in] y_min_inc  y轴 负方向 增量
   */
  void setLimitInc(const double &x_max_inc, const double &x_min_inc,
                   const double &y_max_inc, const double &y_min_inc);
  void setLimitInc(const SpeedDir dir, const double &temp);
  /**
   * @brief 清空偏移量
   *
   */
  void clearLimitInc();

  /**
   * @brief Get the Boundingbox object
   *
   * @param[out] x_max
   * @param[out] x_min
   * @param[out] y_max
   * @param[out] y_min
   */
  void getBoundingbox(double &x_max, double &x_min, double &y_max,
                      double &y_min);

  typedef struct Product {
    Product() = default;
    Product(const Eigen::Vector2d &product, const Eigen::Vector2d &origin) {
      product_ = product;
      origin_ = origin;
      origin_offset_ = origin;
    }
    Eigen::Vector2d product_;
    Eigen::Vector2d origin_;
    Eigen::Vector2d origin_offset_;
  } Product;

 private:
  /**
   * @brief 计算法向量
   *
   */
  void getNormalVector();

  /**
   * @brief 角度限制 -PI~PI
   *
   * @param[in] theta
   * @return double
   */
  double thetaLimit(double theta);

  std::vector<Eigen::Vector2d> v_endpoint_;  // 端点集
  std::vector<Product> v_product_;           // 法向量
                                    // index: point[0,1]  ->  product[0]
  double start_theta_;  // 起始角度
  double end_theta_;    // 结束角度

  Eigen::Vector2d start_product_;  // 起始角度的法向量
  Eigen::Vector2d end_product_;    // 终止角度的法向量

  bool inc_change_flag_ = true;
  double x_max_inc_ = 0;
  double x_min_inc_ = 0;
  double y_max_inc_ = 0;
  double y_min_inc_ = 0;

  double x_max_ = 0;
  double x_min_ = 0;
  double y_max_ = 0;
  double y_min_ = 0;
};

class StopShape {
 public:
  StopShape() = default;
  StopShape(const double &resolution);

  /**
   * @brief 通用push接口
   *
   * @param[in] shape_key   键
   * @param[in] shape_type  形状类型 round->圆 rectangle->矩形
   * @param params          圆  size=3  {center,radius}
   *                        矩形 size=4 {left_top,right_top}
   * @param start_theta     检测角度
   * @param end_theta
   */
  void push(const std::string &shape_key, std::vector<double> params = {},
            double start_theta = 0, double end_theta = 0);
  void push(const StopShapeConfig &config);

  /**
   * @brief 检查点是否在停障框内
   *
   * @param[in] point
   * @param[in] x
   * @param[in] y
   * @return true
   * @return false
   */
  bool checkPonitInside(const Eigen::Vector2d &point, std::string key = "all");
  bool checkPonitInside(const double &x, const double &y,
                        std::string key = "all");
  /**
   * @brief 查点集是否在停障框内 通过Boundingbox 减少查询次数
   *
   * @param[in] v_point         点集
   * @param[out] v_crash_point  发生碰撞的点
   * @param[in] limit_size      触发的点数
   * @return true
   * @return false
   */
  bool checkPonitInside(const std::vector<Eigen::Vector2d> &v_point,
                        std::string key = "all",
                        std::vector<Eigen::Vector2d> *v_crash_point = nullptr,
                        size_t limit_size = 0, bool judge_at_once = false);

  /**
   * @brief Get the Stop Shape object
   *
   * @param[in] key default: all
   * @return std::vector<Eigen::Vector2d>
   */
  std::vector<Eigen::Vector2d> getStopShape(const std::string key = "all");
  void getStopShapeOriginal(std::vector<Eigen::Vector2d> **ptr_v_points,
                            const std::string key = "all");
  /**
   * @brief Set the Resolution object
   *
   * @param[in] resolution 设置分辨率
   */
  void setResolution(const double &resolution);

  /**
   * @brief Set the Inc object
   *
   * @param[in] x_max_inc
   * @param[in] x_min_inc
   * @param[in] y_max_inc
   * @param[in] y_min_inc
   * @param[in] key         default: all
   */
  void setInc(double x_max_inc, double x_min_inc, double y_max_inc,
              double y_min_inc, const std::string key = "all");
  /**
   * @brief Set the Inc object
   *
   * @param[in] dir   增量方向  switch("+x" "-x" "+y" "-y")
   * @param[in] temp  增量
   * @param[in] key   default: all
   */
  void setInc(const SpeedDir dir, const double &temp,
              const std::string key = "all");

  /**
   * @brief 清空所有增量
   *
   */
  void clearInc();

  /**
   * @brief 获取最大包括矩形
   *
   * @param[out] x_max
   * @param[out] x_min
   * @param[out] y_max
   * @param[out] y_min
   * @param[in]  key  default: all
   */
  void getBoundingbox(double &x_max, double &x_min, double &y_max,
                      double &y_min, const std::string key = "all");

 private:
  /**
   * @brief Set the Stop Shape object
   *
   * @param[in] key default: all
   * @return std::vector<Eigen::Vector2d>
   */
  std::vector<Eigen::Vector2d> CalculateStopShape(
      const std::string key = "all");

  // 可以存在多个不同形状的停障框
  std::multimap<std::string, StopPolygon> m_polygon_;  // 停障框
  double resolution_ = 0;                              // 分辨率

  std::map<std::string, std::vector<Eigen::Vector2d>> m_dir_shape_point_;
  std::map<std::string, std::vector<double>> m_inc_;
};
}  // namespace CVTE_BABOT
#endif