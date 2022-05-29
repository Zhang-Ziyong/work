/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file costmap_2d.hpp
 *
 *@brief
 * 1.用于存储2d costmap代价数据
 * 2.作为costmap_layer的基类
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-03
 ************************************************************************/
#ifndef __COSTMAP2D_HPP
#define __COSTMAP2D_HPP

#include <limits.h>
#include <math.h>
#include <memory.h>
#include <stdio.h>
#include <boost/smart_ptr.hpp>
#include <memory>
#include <mutex>
#include <vector>

#include "costmap_utils.hpp"
namespace CVTE_BABOT {
/**
 * Costmap2d
 * @brief
 * 1.对costmap公共操作
 * 2. 构造代价地图的基类
 **/

class Costmap2d {
 public:
  Costmap2d();
  Costmap2d(const Costmap2d &map);
  Costmap2d &operator=(const Costmap2d &map);

  virtual ~Costmap2d() = default;
  /**
   *Costmap2d
   *@brief
   *初始化Costmap2d
   *
   *@param[in] ui_cells_size_x-栅格x坐标大小
   *@param[in] ui_cells_size_y-栅格y坐标大小
   *@param[in] d_resolution-栅格分辨率
   *@param[in] d_origin_x-地图原点x坐标
   *@param[in] d_origin_y-地图原点y坐标
   *@param[in] uc_default_value-栅格代价值
   *@return 具体的代价值
   * */
  Costmap2d(const unsigned int &ui_cells_size_x,
            const unsigned int &ui_cells_size_y, const double &d_resolution,
            const double &d_origin_x, const double &d_origin_y,
            const unsigned char &uc_default_value = 0);

  /**
   *getOriginX
   *@brief
   *获得costmap的原点的x坐标
   *
   *@return origin_x_-x坐标原点数值
   * */
  inline double getOriginX() const { return d_origin_x_; }

  /**
   *getOriginY
   *@brief
   *获得costmap的原点的y坐标
   *
   *@return origin_y_-y坐标原点数值
   * */
  inline double getOriginY() const { return d_origin_y_; }

  /**
   *getResolution
   *@brief
   *获得costmap的分辨率
   *
   *@return resolution_-分辨率数值数值，单位m
   * */
  inline double getResolution() const { return d_resolution_; }

  /**
   *getSizeInMetersX
   *@brief
   *获得costmap的x轴的大小
   *
   *@return x轴的大小,单位m
   * */
  inline double getSizeInMetersX() const {
    return (ui_size_x_ - 1 + 0.5) * d_resolution_;
  }

  inline double getMapSizeInMetersX(const double &size_x) const {
    return (size_x - 1 + 0.5) * d_resolution_;
  }

  inline double getMapSizeInMetersY(const double &size_y) const {
    return (size_y - 1 + 0.5) * d_resolution_;
  }

  /**
   *getResolution
   *@brief
   *获得costmap的y轴的大小
   *
   *@return y轴的大小,单位m
   * */
  inline double getSizeInMetersY() const {
    return (ui_size_y_ - 1 + 0.5) * d_resolution_;
  }

  /**
   *getResolution
   *@brief
   *获得costmap的x轴的栅格大小
   *
   *@return x轴的栅格大小,单位个
   * */
  inline unsigned int getSizeInCellsX() const { return ui_size_x_; };

  /**
   *getResolution
   *@brief
   *获得costmap的y轴的栅格大小
   *
   *@return y轴的栅格大小,单位个
   * */
  inline unsigned int getSizeInCellsY() const { return ui_size_y_; }

  /**
   *cellDistance
   *@brief
   *将世界地图的距离转换成栅格地图距离
   *
   *@param[in] d_world_dist-世界地图距离，单位m
   *@return -costmap栅格地图距离，单位个
   * */
  inline unsigned int cellDistance(const double &d_world_dist) const {
    double cells_dist = std::max(0.0, ceil(d_world_dist / d_resolution_));
    unsigned int ui = static_cast<unsigned int>(cells_dist);
    return ui;
  }

  /**
   *getCharMap
   *@brief
   *获得costmap的所有栅格代价
   *
   *@return ptr_costmap_-代价值智能指针，地图代价值矩阵
   * */
  inline boost::shared_array<unsigned char> getCharMap() const {
    return ptr_uc_costmap_;
  }

  /**
   *getMutx
   *@brief
   *获得costmap的递归锁指针
   *
   *@return ptr_mutex_access_-costmap的递归锁指针
   * */
  inline std::shared_ptr<std::recursive_mutex> getMutx() const {
    return ptr_mutex_access_;
  }

  /**
   *getCost
   *@brief
   *获得costmap的某个栅格的代价值
   *
   *@param[in] ui_mx-栅格x坐标
   *@param[in] ui_my-栅格y坐标
   *@return 具体的代价值
   * */
  inline unsigned char getCost(const CostmapPoint &cp_point) const {
    return ptr_uc_costmap_[getIndex(cp_point.ui_x, cp_point.ui_y)];
  }

  /**
   *getCost
   *@brief
   *获得costmap的某个栅格的代价值
   *
   *@param[in] ui_mx-栅格x坐标
   *@param[in] ui_my-栅格y坐标
   *@return 具体的代价值
   * */
  inline unsigned char getCost(const unsigned int &ui_mx,
                               const unsigned int &ui_my) const {
    return ptr_uc_costmap_[getIndex(ui_mx, ui_my)];
  }

  /**
   *getCostWithMapPose
   *@brief
   *获得costmap的某个栅格的代价值,输入值是世界坐标系下的点
   *
   *@param[in] point_x-世界x坐标
   *@param[in] point_y-世界y坐标
   *@return 具体的代价值
   * */
  int getCostWithMapPose(const double &point_x, const double &point_y);

  /**
   *setCost
   *@brief
   *设置costmap的某个栅格的代价值
   *
   *@param[in] ui_mx-栅格x坐标
   *@param[in] ui_my-栅格y坐标
   *@param[in] uc_cost-代价值
   * */
  inline void setCost(const unsigned int &ui_mx, const unsigned int &ui_my,
                      const unsigned char &uc_cost) {
    ptr_uc_costmap_[getIndex(ui_mx, ui_my)] = uc_cost;
  }

  /**
   *mapToWorld
   *@brief
   *将costmap的栅格坐标转化成世界地图坐标
   *
   *@param[in] cp_point-栅格x，y坐标
   *@param[out] wm_point-世界地图x，y坐标
   *
   * */
  void mapToWorld(const CostmapPoint &cp_point, WorldmapPoint &wm_point) const;

  /**
   *worldToMap
   *@brief
   *将世界地图坐标转化成costmap的栅格坐标(有边界限制)
   *
   *@param[in] wm_point-世界地图x，y坐标
   *@param[out] cp_point-栅格x，y坐标
   *@return true-表示栅格坐标在costmap的范围里面，false-表示超出了costmap范围
   * */
  bool worldToMap(const WorldmapPoint &wm_point, CostmapPoint &cp_point) const;

  /**
   *worldToMapNoBounds
   *@brief
   *将世界地图坐标转化成costmap的栅格坐标(没有边界限制)
   *
   *@param[in] d_wx-世界地图x坐标
   *@param[in] d_wx-世界地图y坐标
   *@param[out] ui_mx-栅格x坐标
   *@param[out] ui_my-栅格y坐标
   * */
  void worldToMapNoBounds(const double &d_wx, const double &d_wy, int &i_mx,
                          int &i_my) const;

  /**
   *worldToMapEnforceBounds
   *@brief
   *将世界地图坐标转化成costmap的栅格坐标(强制转换到边界)
   *
   *@param[in] d_wx-世界地图x坐标
   *@param[in] d_wx-世界地图y坐标
   *@param[out] ui_mx-栅格x坐标
   *@param[out] ui_my-栅格y坐标
   * */
  void worldToMapEnforceBounds(const double &d_wx, const double &d_wy,
                               int &i_mx, int &i_my) const;

  /**
   *getIndex
   *@brief
   *已知costmap的坐标，计算在一位数组的下标
   *
   *@param[in] ui_mx-x坐标
   *@param[in] ui_my-y坐标
   *@return 在一维数组的下标
   * */
  inline unsigned int getIndex(const unsigned int &ui_mx,
                               const unsigned int &ui_my) const {
    return ui_my * ui_size_x_ + ui_mx;
  }

  /**
   *indexToCells
   *@brief
   *已知在一位数组的下标，计算costmap的栅格坐标
   *
   *@param[in] ui_index-一维数组的下标
   *@param[out] ui_mx-x坐标
   *@param[out] ui_my-y坐标
   * */
  inline void indexToCells(const unsigned int &ui_index, unsigned int &ui_mx,
                           unsigned int &ui_my) const {
    ui_my = ui_index / ui_size_x_;
    ui_mx = ui_index - (ui_my * ui_size_x_);
  }

  /**
   *setDefaultValue
   *@brief
   *设置costmap的默认代价值
   *
   *@param[in] uc_c-默认代价值
   * */
  inline void setDefaultValue(const unsigned char &uc_c) {
    uc_default_value_ = uc_c;
  }

  /**
   *getDefaultValue
   *@brief
   *获得默认的代价值
   *
   *@return uc_default_value_-默认代价值
   * */
  inline unsigned char getDefaultValue() const { return uc_default_value_; }

  /**
   *setConvexPolygonCost
   *@brief
   *设置机器人多变形的内的栅格的代价值
   *
   *@param[in] polygon-机器人的多边形
   *@param[in] cost_value-代价值
   *@return true-设置成功，false-设置失败(超出costmap大小)
   * */
  bool setConvexPolygonCost(const std::vector<WorldmapPoint> &polygon,
                            const unsigned char &cost_value);
  /**
   *polygonOutlineCells
   *@brief
   *已知机器人多边形的几个端点，计算出端点连线的所有点
   *
   *@param[in] polygon-机器人的多边形端点
   *@param[out] polygon_cells-机器人的多边形l轮廓所有的点
   *@return true-计算成功，false-计算失败
   * */
  bool polygonOutlineCells(const std::vector<CostmapPoint> &polygon,
                           std::vector<CostmapPoint> &polygon_cells);

  /**
   *convexFillCells
   *@brief
   *已知机器人多边形的几个端点，计算出多边形内的所有点
   *
   *@param[in] polygon-机器人的多边形端点
   *@param[out] polygon_cells-机器人的多边形内所有的点
   *@return true-计算成功，false-计算失败
   * */
  bool convexFillCells(const std::vector<CostmapPoint> &polygon,
                       std::vector<CostmapPoint> &polygon_cells);
  /**
   * @brief 合并两个代价地图, 合并过程取最大的代价值
   *
   * @param ptr_local_costmap 输入的代价地图
   */
  void combineCostmap(std::shared_ptr<Costmap2d> ptr_local_costmap);

  /**
   *updateOrigin
   *@brief
   *更新costmap原点
   *
   *@param[in] d_new_origin_x-原点x轴坐标
   *@param[in] d_new_origin_y-原点y轴坐标
   * */
  virtual void updateOrigin(const double &d_new_origin_x,
                            const double &d_new_origin_y);

  /**
   *resizeMap
   *@brief
   *重新设置costmap大小、分辨率、原点
   *
   *@param[in] ui_size_x-x轴大小
   *@param[in] ui_size_y-y轴大小
   *@param[in] d_resolution-分辨率
   *@param[in] d_origin_x-原点x轴坐标
   *@param[in] d_origin_y-原点y轴坐标
   * */
  void resizeMap(const unsigned int &ui_size_x, const unsigned int &ui_size_y,
                 const double &d_resolution, const double &d_origin_x,
                 const double &d_origin_y);

  /**
   *resetMap
   *@brief
   *对costmap部分区域代价值赋值为默认代价
   *
   *@param[in] ui_x0-x轴起点
   *@param[in] ui_y0-y轴起点
   *@param[in] ui_xn-x轴起点
   *@param[in] ui_yn-y轴起点
   * */
  void resetMap(const unsigned int &ui_x0, const unsigned int &ui_y0,
                const unsigned int &ui_xn, const unsigned int &ui_yn);

  /**
   *computeLineCells
   *@brief
   *计算两点间的栅格坐标, 给obstacle_map使用, 以后考虑放到obstacle_map中
   *
   *@param[in] x0-起始点x坐标
   *@param[in] y0-起始点y坐标
   *@param[in] x1-终点x坐标
   *@param[in] y1-终点y坐标
   *@param[out] v_line_points-两点经过的栅格坐标
   **/
  void computeLineCells(const unsigned int &x0, const unsigned int &y0,
                        const unsigned int &x1, const unsigned int &y1,
                        std::vector<CostmapPoint> &v_line_points);

  typedef std::shared_ptr<Costmap2d>
      Costmap2dPtr;  ///< 声明一个Costmap智能指针类型

 private:
  std::shared_ptr<std::recursive_mutex> ptr_mutex_access_ = nullptr;
  ///<替归锁,保护数据安全

  /**
   *sign
   *@brief
   *用实现一个阶跃函数
   *
   *@param[in] x-阶跃函数的输入
   *@return 阶跃函数的输出
   * */
  inline int sign(const int &x) const { return x > 0 ? 1.0 : -1.0; }

  /**
   *bresenham2D
   *@brief
   *用Bresenham算法(布雷森汉姆)计算直线上所以的点，给raytraceLine调用
   *
   *@param[out] at-操作的对象模板
   *@param[in] abs_da-两点在a轴上的偏差的绝对值
   *@param[in] abs_db-两点在b轴上的偏差的绝对值
   *@param[in] offset_a-a轴上的偏差的绝对值的阶跃输出
   *@param[in] offset_b-b轴上的偏差的绝对值的阶跃输出 * 对应地图的宽度
   *@param[in] offset- 偏移地图原点的大小
   *@param[in] max_length- 最大距离
   * */
  template <class ActionType>
  void bresenham2D(ActionType at, const unsigned int abs_da,
                   const unsigned int abs_db, int error_b, const int offset_a,
                   const int offset_b, unsigned int offset,
                   const unsigned int max_length) {
    unsigned int end = std::min(max_length, abs_da);
    for (unsigned int i = 0; i < end; ++i) {
      at(offset);
      offset += offset_a;
      error_b += abs_db;
      if (static_cast<unsigned int>(error_b) >= abs_da) {
        offset += offset_b;
        error_b -= abs_da;
      }
    }
    at(offset);
  }

 protected:
  double d_origin_x_ = 0.0;  ///< costmap的起点，不是中点，在全局地图的坐标
  double d_origin_y_ = 0.0;  ///< costmap的起点，不是中点，在全局地图的坐标
  double d_resolution_ = 0.0;   ///< 地图分辨率
  unsigned int ui_size_x_ = 0;  ///< 地图X方向宽度
  unsigned int ui_size_y_ = 0;  ///< 地图X方向宽度

  unsigned char uc_default_value_ = 0;  ///< 默认灰度值
  boost::shared_array<unsigned char> ptr_uc_costmap_ =
      nullptr;  ///< 地图灰度值矩阵,使用shared_array智能指针

  /**
   *resetMaps
   *@brief
   *将costmap重新赋值默认代价
   *
   * */
  virtual void resetMaps();

  /**
   *initMaps
   *@brief
   *初始化costmap,根据大小
   *
   *@param[in] ui_size_x-x轴大小
   *@param[in] ui_size_y-y轴大小
   * */
  virtual void initMaps(const unsigned int &ui_size_x,
                        const unsigned int &ui_size_y);

  /**
   *copyMapRegion
   *@brief
   *复制地图部分区域,传入指针
   *
   *@param[in] source_map-原地图
   *@param[in] sm_lower_left_x-原地图开始复制的x轴起点，左下方的
   *@param[in] sm_lower_left_y-原地图开始复制的y轴起点，左下方的
   *@param[in] sm_size_x-原地图x轴大小
   *@param[in] dest_map-目标地图
   *@param[in] dm_lower_left_x-目标地图开始复制的x轴起点，左下方的
   *@param[in] dm_lower_left_y-目标地图开始复制的y轴起点，左下方的
   *@param[in] dm_size_x-目标地图x轴大小
   *@param[in] region_size_x-复制x轴的大小
   *@param[in] region_size_y-复制y轴的大小
   * */
  template <typename data_type>
  void copyMapRegionRaw(data_type *source_map,
                        const unsigned int &sm_lower_left_x,
                        const unsigned int &sm_lower_left_y,
                        const unsigned int &sm_size_x, data_type *dest_map,
                        const unsigned int &dm_lower_left_x,
                        const unsigned int &dm_lower_left_y,
                        const unsigned int &dm_size_x,
                        const unsigned int &region_size_x,
                        const unsigned int &region_size_y) const {
    // we'll first need to compute the starting points for each map
    data_type *sm_index =
        source_map + (sm_lower_left_y * sm_size_x + sm_lower_left_x);
    data_type *dm_index =
        dest_map + (dm_lower_left_y * dm_size_x + dm_lower_left_x);

    // now, we'll copy the source map into the destination map
    for (unsigned int i = 0; i < region_size_y; ++i) {
      memcpy(dm_index, sm_index, region_size_x * sizeof(data_type));
      sm_index += sm_size_x;
      dm_index += dm_size_x;
    }
  }

  /**
   *copyMapRegion
   *@brief
   *复制地图部分区域，传入智能数组
   *
   *@param[in] source_map-原地图
   *@param[in] sm_lower_left_x-原地图开始复制的x轴起点，左下方的
   *@param[in] sm_lower_left_y-原地图开始复制的y轴起点，左下方的
   *@param[in] sm_size_x-原地图x轴大小
   *@param[in] dest_map-目标地图
   *@param[in] dm_lower_left_x-目标地图开始复制的x轴起点，左下方的
   *@param[in] dm_lower_left_y-目标地图开始复制的y轴起点，左下方的
   *@param[in] dm_size_x-目标地图x轴大小
   *@param[in] region_size_x-复制x轴的大小
   *@param[in] region_size_y-复制y轴的大小
   * */
  template <typename data_type>
  void copyMapRegion(
      const boost::shared_array<data_type> &source_map,
      const unsigned int &sm_lower_left_x, const unsigned int &sm_lower_left_y,
      const unsigned int &sm_size_x, boost::shared_array<data_type> &dest_map,
      const unsigned int &dm_lower_left_x, const unsigned int &dm_lower_left_y,
      const unsigned int &dm_size_x, const unsigned int &region_size_x,
      const unsigned int &region_size_y) const {
    // we'll first need to compute the starting points for each map
    data_type *sm_index =
        source_map.get() + (sm_lower_left_y * sm_size_x + sm_lower_left_x);
    data_type *dm_index =
        dest_map.get() + (dm_lower_left_y * dm_size_x + dm_lower_left_x);

    // now, we'll copy the source map into the destination map
    for (unsigned int i = 0; i < region_size_y; ++i) {
      memcpy(dm_index, sm_index, region_size_x * sizeof(data_type));
      sm_index += sm_size_x;
      dm_index += dm_size_x;
    }
  }

  /**
   *raytraceLine
   *@brief
   *计算两点直线中的所有点
   *
   *@param[in&out] at-操作的对象模板
   *@param[in] x0-直线起点x
   *@param[in] y0-直线起点y
   *@param[in] x0-直线终点x
   *@param[in] y0-直线终点y
   *@param[in] max_length-最大长度
   * */
  template <class ActionType>
  inline void raytraceLine(ActionType at, const unsigned int &x0,
                           const unsigned int &y0, const unsigned int &x1,
                           const unsigned int &y1,
                           unsigned int max_length = UINT_MAX) {
    int dx = x1 - x0;
    int dy = y1 - y0;

    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);

    int offset_dx = sign(dx);
    int offset_dy = sign(dy) * ui_size_x_;

    unsigned int offset = y0 * ui_size_x_ + x0;

    double dist = hypot(dx, dy);
    double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);

    if (abs_dx >= abs_dy) {
      int error_y = abs_dx / 2;
      bresenham2D(at, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset,
                  static_cast<unsigned int>(scale * abs_dx));
      return;
    }

    int error_x = abs_dy / 2;
    bresenham2D(at, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset,
                static_cast<unsigned int>(scale * abs_dy));
  }

  /**
   *MarkCell
   *@brief
   *costmap生成直线中的栅格并赋值，在计算射线栅格中使用
   *
   *@param[in&out] ptr_costmap-操作的costmap
   *@param[in] value-代价值
   * */
  class MarkCell {
   public:
    MarkCell(boost::shared_array<unsigned char> ptr_costmap,
             const unsigned char &value)
        : ptr_costmap_(ptr_costmap), value_(value) {}
    inline void operator()(unsigned int offset) {
      ptr_costmap_[offset] = value_;
    }

   private:
    boost::shared_array<unsigned char> ptr_costmap_;  ///< 存储costmap
    unsigned char value_;                             ///< 代价值
  };

  /**
   *PolygonOutlineCells
   *@brief
   *在costmap中生成机器人多边形直线中的栅格并赋值
   *
   *@param[in&out] ptr_costmap-操作的costmap
   *@param[in] costmap-操作的costmap2d
   *@param[in] cells-机器人多边形的点
   * */
  class PolygonOutlineCells {
   public:
    PolygonOutlineCells(const Costmap2d &costmap,
                        const boost::shared_array<unsigned char> &ptr_map,
                        std::vector<CostmapPoint> &cells)
        : costmap_(costmap), ptr_map_(ptr_map), cells_(cells) {}
    // just push the relevant cells back onto the list
    inline void operator()(unsigned int offset) {
      CostmapPoint loc;
      costmap_.indexToCells(offset, loc.ui_x, loc.ui_y);
      cells_.push_back(loc);
    }

   private:
    const Costmap2d &costmap_;                    ///< costmap2d 对象
    boost::shared_array<unsigned char> ptr_map_;  ///< 存储costmap代价
    std::vector<CostmapPoint> &cells_;  ///< 存储机器人多边形的点
  };
};

}  // namespace CVTE_BABOT

#endif