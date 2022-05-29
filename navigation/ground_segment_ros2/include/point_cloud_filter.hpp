/**
 * @file Recon_filter.h
 * @Mingzi Tao(i_taoziming@cvte.com)
 * @brief
 *  *  该段代码定义了类“Point_Cloud_Filter”的声明，用来完成点云的预处理与滤波。
 *  *
 * 类“Point_Cloud_Cluster”包含两个成员，PointCloud_Ptr_保存待处理的点云数据，Point_Index_保存滤波的结果。
 *  *  类的构造函数仅赋予PointCloud_Ptr_空指针。
 *  *  定义了六个功能函数：setInputCloud对PointCloud_Ptr_赋值，
 *                      passPointFilter直通滤波，选定点云数据的处理范围，
 *                      voxelGridFilter对点云进行降采样，即体素化，
 *                      heightDifferenceFilter根据高度差及其均方差进行条件滤波，
 *                      smoothNeighborFilter定义光滑程度，并以此条件滤波，
 * @version 0.1
 * @date 2019-07-23
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef RECON_FILTER_H
#define RECON_FILTER_H

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string.h>

class Point_Cloud_Filter {
 public:
  /**
*@brief 待聚类的点云目标。
*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud_Ptr_;

  /**
*@brief
* * 根据输入的点云记录滤波的结果。
* * 以vector的形式顺序记录滤波结果的点云序列号。
*/
  std::vector<int> Point_Index_;

  Point_Cloud_Filter(const Point_Cloud_Filter &Point_Cloud_Filter) = delete;
  Point_Cloud_Filter &operator=(const Point_Cloud_Filter &Point_Cloud_Filter) =
      delete;

  Point_Cloud_Filter();
  ~Point_Cloud_Filter();

  /**
*@brief Set the Input Point Cloud object
*
*@param Input_PointCloud_Ptr
*/
  void setInputCloud(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr Input_PointCloud_Ptr);

  /**
*@brief
*根据高度差及其均方差进行条件滤波。主要有三个阈值参数，高度差的Threshold_1,
*       Threshold_2和Threshold_3。该函数直接对类成员PointCloud_Ptr_操作。
*
*@param kdtree 待处理点云的KD树
*@param radius KD树的搜索半径
*@param Threshold_1 Threshold_1 < Z_max-Z_min
*@param Threshold_2 Z_max-Z_min < Threshold_2
*@param Threshold_3 Threshold_3 < Z_var
*/
  void heightDifferenceFilter(const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree,
                              const float &radius, const float &Threshold_1,
                              const float &Threshold_2,
                              const float &Threshold_3);

  /**
*@brief
*定义光滑程度，并以此条件滤波。主要有一个阈值参数Thsmooth。该函数直接对类成员PointCloud_Ptr_操作。
*
*@param kdtree 待处理点云的KD树
*@param radius KD树的搜索半径
*@param Thsmooth Thsmooth < Z_smooth
*/
  void smoothNeighborFilter(const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree,
                            const float &radius, const float &Thsmooth);

  /**
*@brief 直通滤波，选定点云数据的处理范围
*
*@param limits_begin 处理范围的下限
*@param limits_end 处理范围的上限
*@param coordinate 选择处理的坐标系，仅能填写“x”，“y”，和“z”
*@param Filter_PointCloud 输出处理后的结果
*/
  void passPointFilter(const float &limits_begin, const float &limits_end,
                       const std::string &coordinate,
                       pcl::PointCloud<pcl::PointXYZ> &Filter_PointCloud);

  /**
*@brief 该函数通过体素化网格对点云降采样
*
*@param leafsize_x x轴方向的体素化大小
*@param leafsize_y y轴方向的体素化大小
*@param leafsize_z z轴方向的体素化大小
*@param Filter_PointCloud 输出处理后的结果
*/
  void voxelGridFilter(const float &leafsize_x, const float &leafsize_y,
                       const float &leafsize_z,
                       pcl::PointCloud<pcl::PointXYZ> &Filter_PointCloud);
};

typedef std::shared_ptr<Point_Cloud_Filter> Point_Cloud_FilterPtr;

#endif