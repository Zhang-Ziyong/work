/**
 * @file Recon_cluster.hpp
 * @Mingzi Tao(i_taoziming@cvte.com)
 * @brief Recon_cluster.h中定义的类的具体实现
 * @version 0.1
 * @date 2019-07-23
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "point_cloud_filter.hpp"

/**
 * @brief Construct a new Point_Cloud_Cluster::Point_Cloud_Cluster object
 *        对类成员变量赋予空指针
 */
Point_Cloud_Filter::Point_Cloud_Filter() { PointCloud_Ptr_ = nullptr; }

Point_Cloud_Filter::~Point_Cloud_Filter() {}

/**
 * @brief 为成员变量赋值
 *
 * @param Input_PointCloud_Ptr 输入的待处理点云
 */
void Point_Cloud_Filter::setInputCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr Input_PointCloud_Ptr) {
  PointCloud_Ptr_ = Input_PointCloud_Ptr;
}

/**
 * @brief
 * 根据高度差及其均方差进行条件滤波。主要有三个阈值参数，高度差的Threshold_1,
 *        Threshold_2和Threshold_3。该函数直接对类成员PointCloud_Ptr_操作。
 *
 * @param kdtree 待处理点云的KD树
 * @param radius KD树的搜索半径
 * @param Threshold_1 Threshold_1 < Z_max-Z_min
 * @param Threshold_2 Z_max-Z_min < Threshold_2
 * @param Threshold_3 Threshold_3 < Z_var
 */
void Point_Cloud_Filter::heightDifferenceFilter(
    const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, const float &radius,
    const float &Threshold_1, const float &Threshold_2,
    const float &Threshold_3) {
  Point_Index_.clear();

  /**
*@brief pointIdxRadiusSearch：点P的邻域之内的点的序列集
* *  *  pointRadiusSquaredDistance：邻域之内的点到点P的距离
*/
  std::vector<int> pointIdxSearch;
  std::vector<float> pointRadiusSquaredDistance;

  /**
*@brief 遍历点云中的所有点，搜索所有点的邻域，根据准则判断是否符合要求
*/
  for (size_t i = 0; i < PointCloud_Ptr_->points.size(); i++) {
    /**
* @brief 搜索待处理点云建立的KD树，获取其邻域内点的序列号
*/
    kdtree.radiusSearch(PointCloud_Ptr_->points[i], radius, pointIdxSearch,
                        pointRadiusSquaredDistance);
    float Z_max, Z_min, Z_u = 0;
    float Z_sum = 0;

    /**
* @brief 计算Z_max, Z_min, Z_u和Z_var
*/
    Z_max = PointCloud_Ptr_->points[i].z;
    Z_min = PointCloud_Ptr_->points[i].z;
    for (size_t j = 0; j < pointIdxSearch.size(); j++) {
      if (Z_max < PointCloud_Ptr_->points[pointIdxSearch[j]].z) {
        Z_max = PointCloud_Ptr_->points[pointIdxSearch[j]].z;
      }
      if (Z_min > PointCloud_Ptr_->points[pointIdxSearch[j]].z) {
        Z_min = PointCloud_Ptr_->points[pointIdxSearch[j]].z;
      }
      Z_sum = Z_sum + PointCloud_Ptr_->points[pointIdxSearch[j]].z;
    }
    Z_u = Z_sum / pointIdxSearch.size();

    float Z_var = 0.0;
    for (size_t j = 0; j < pointIdxSearch.size(); j++) {
      Z_var = Z_var +
              (PointCloud_Ptr_->points[pointIdxSearch[j]].z - Z_u) *
                  (PointCloud_Ptr_->points[pointIdxSearch[j]].z - Z_u);
    }
    Z_var = sqrt(Z_var / pointIdxSearch.size());

    // ROS_INFO("The Z_Var is: %f", Z_var);

    /**
* @brief 激光雷达自身高度设定为某一值，该点云中激光雷达的高度大约在0.6，
*  *  *  可认为1.2以下的点云需要识别。另外实际上需要考虑机器人高度。
*  *  *  Threshold_1 < Z_max-Z_min
*  *  *  Z_max-Z_min < Threshold_2
*  *  *  Threshold_3 < Z_var
*/
    if (PointCloud_Ptr_->points[i].z < 0.6 && Threshold_1 < (Z_max - Z_min) &&
        (Z_max - Z_min) < Threshold_2 && Z_var > Threshold_3) {
      Point_Index_.push_back(i);
    }
  }

  pcl::PointIndices indices_R_H_F;

  /**
*@brief initialExtractIndices cliper
* *  *  该类为pcl库中用于剪切点云数据集的类
* *  *  setIndices设置点云的序列号
* *  *  setNegative---false表示保留序列点，true表示保留序列外点
*/
  pcl::ExtractIndices<pcl::PointXYZ> cliper;

  cliper.setInputCloud(PointCloud_Ptr_);
  for (size_t i = 0; i < Point_Index_.size(); i++) {
    indices_R_H_F.indices.push_back(Point_Index_[i]);
  }
  cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices_R_H_F));
  cliper.setNegative(false);
  cliper.filter(*PointCloud_Ptr_);
}

/**
 * @brief
 * 定义光滑程度，并以此条件滤波。主要有一个阈值参数Thsmooth。该函数直接对类成员PointCloud_Ptr_操作。
 *
 * @param kdtree 待处理点云的KD树
 * @param radius KD树的搜索半径
 * @param Thsmooth Thsmooth < Z_smooth
 */
void Point_Cloud_Filter::smoothNeighborFilter(
    const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, const float &radius,
    const float &Thsmooth) {
  Point_Index_.clear();

  /**
*@brief pointIdxRadiusSearch：点P的邻域之内的点的序列集
* *  *  pointRadiusSquaredDistance：邻域之内的点到点P的距离
*/
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  /**
*@brief 遍历点云中的所有点，搜索所有点的邻域，根据准则判断是否符合要求
*/
  for (size_t i = 0; i < PointCloud_Ptr_->points.size(); i++) {
    float smooth;

    /**
* @brief 搜索待处理点云建立的KD树，获取其邻域内点的序列号
*/
    kdtree.radiusSearch(PointCloud_Ptr_->points[i], radius,
                        pointIdxRadiusSearch, pointRadiusSquaredDistance);
    int S = pointIdxRadiusSearch.size();

    /**
* @brief
* 如果存在某一点的其邻域内不存在任何点，认为该点为噪点，记其光滑度为0
*/
    if (S == 0) {
      smooth = 0;
      continue;
    }

    /**
* @brief P_coordinate表示该点的坐标，且计算邻域内的光滑度
*  *  *  光滑度的定义为邻域内所有点到P_coordinate的向量之和的模值
*/
    pcl::PointXYZ P_coordinate = PointCloud_Ptr_->points[i];
    pcl::PointXYZ Sum_vector;
    Sum_vector.x = 0;
    Sum_vector.y = 0;
    Sum_vector.z = 0;
    for (size_t j = 0; j < pointIdxRadiusSearch.size(); j++) {
      Sum_vector.x = Sum_vector.x + P_coordinate.x -
                     PointCloud_Ptr_->points[pointIdxRadiusSearch[j]].x;
      Sum_vector.y = Sum_vector.y + P_coordinate.y -
                     PointCloud_Ptr_->points[pointIdxRadiusSearch[j]].y;
      Sum_vector.z = Sum_vector.z + P_coordinate.z -
                     PointCloud_Ptr_->points[pointIdxRadiusSearch[j]].z;
    }

    /**
* @brief 计算光滑度的值
*/
    float sum_vector_2fan =
        sqrt((Sum_vector.x * Sum_vector.x) + (Sum_vector.y * Sum_vector.y) +
             (Sum_vector.z * Sum_vector.z));
    smooth = sum_vector_2fan / S;

    // ROS_INFO("The Points Smooth is: %f", smooth);

    if (smooth >= Thsmooth) {
      Point_Index_.push_back(i);
    }
  }

  pcl::PointIndices indices_R_S_F;

  /**
*@brief initialExtractIndices cliper
* *  *  该类为pcl库中用于剪切点云数据集的类
* *  *  setIndices设置点云的序列号
* *  *  setNegative---false表示保留序列点，true表示保留序列外点
*/
  pcl::ExtractIndices<pcl::PointXYZ> cliper;
  cliper.setInputCloud(PointCloud_Ptr_);
  for (size_t i = 0; i < Point_Index_.size(); i++) {
    indices_R_S_F.indices.push_back(Point_Index_[i]);
  }
  cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices_R_S_F));
  cliper.setNegative(false);
  cliper.filter(*PointCloud_Ptr_);
}

/**
 * @brief 直通滤波，选定点云数据的处理范围
 *
 * @param limits_begin 处理范围的下限
 * @param limits_end 处理范围的上限
 * @param coordinate 选择处理的坐标系，仅能填写“x”，“y”，和“z”
 * @param Filter_PointCloud 输出处理后的结果
 */
void Point_Cloud_Filter::passPointFilter(
    const float &limits_begin, const float &limits_end,
    const std::string &coordinate,
    pcl::PointCloud<pcl::PointXYZ> &Filter_PointCloud) {
  /**
*@brief pcl::PassThrough<pcl::PointXYZ> pcl库自带的直通滤波处理的类
*/
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(PointCloud_Ptr_);
  pass.setFilterFieldName(coordinate);
  pass.setFilterLimits(limits_begin, limits_end);
  pass.filter(Filter_PointCloud);
}

/**
 * @brief 该函数通过体素化网格对点云降采样
 *
 * @param leafsize_x x轴方向的体素化大小
 * @param leafsize_y y轴方向的体素化大小
 * @param leafsize_z z轴方向的体素化大小
 * @param Filter_PointCloud 输出处理后的结果
 */
void Point_Cloud_Filter::voxelGridFilter(
    const float &leafsize_x, const float &leafsize_y, const float &leafsize_z,
    pcl::PointCloud<pcl::PointXYZ> &Filter_PointCloud) {
  /**
*@brief pcl::VoxelGrid<pcl::PointXYZ> pcl库自带的体素化网格处理的类
*/
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(PointCloud_Ptr_);
  sor.setLeafSize(leafsize_x, leafsize_y, leafsize_z);
  sor.filter(Filter_PointCloud);
}