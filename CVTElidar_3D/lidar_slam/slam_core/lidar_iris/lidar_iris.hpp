/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file lidar_iris.hpp
 *
 *@brief
 * 激光虹膜特征提取和比较
 *
 *@modified by caoyong(caoyong@cvte.com)
 *
 *@author caoyong(caoyong@cvte.com)
 *@version V1.0
 *@data 2020-11-03
 ************************************************************************/

#ifndef LIADR_IRIS_HPP
#define LIADR_IRIS_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <flann/flann.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace cvte_lidar_slam {

class LidarIris {
 public:
  struct FeatureDesc {
    cv::Mat1b img;
    cv::Mat1b T;
    cv::Mat1b M;
  };

  LidarIris(int n_scale, int min_wave_length, float mult, float sigma_onf,
            int match_num)
      : n_scale_(n_scale),
        min_wave_length_(min_wave_length),
        mult_(mult),
        sigma_onf_(sigma_onf),
        match_num_(match_num),
        vec_list_(flann::Index<flann::L2<float>>(flann::KDTreeIndexParams(4))),
        indices_(flann::Matrix<int>(new int[match_num_], 1, match_num_)),
        dists_(flann::Matrix<float>(new float[match_num_], 1, match_num_)) {}
  LidarIris(const LidarIris &) = delete;
  LidarIris &operator=(const LidarIris &) = delete;

  static cv::Mat1b getIris(const pcl::PointCloud<pcl::PointXYZI> &cloud);
  //
  void updateFrame(const cv::Mat1b &frame, int frame_index,
                   float *match_distance, int *match_index);
  //
  float compare(const FeatureDesc &img1, const FeatureDesc &img2,
                int *bias = nullptr);
  //
  FeatureDesc getFeature(const cv::Mat1b &src);
  FeatureDesc getFeature(const cv::Mat1b &src, std::vector<float> &vec);
  std::vector<cv::Mat2f> logGaborFilter(const cv::Mat1f &src,
                                        unsigned int n_scale,
                                        int min_wave_length, double mult,
                                        double sigma_onf);
  void getHammingDistance(const cv::Mat1b &T1, const cv::Mat1b &M1,
                          const cv::Mat1b &T2, const cv::Mat1b &M2, int scale,
                          float &dis, int &bias);
  //
  static inline cv::Mat circRowShift(const cv::Mat &src, int shift_m_rows);
  static inline cv::Mat circColShift(const cv::Mat &src, int shift_n_cols);
  static cv::Mat circShift(const cv::Mat &src, int shift_m_rows,
                           int shift_n_cols);

 private:
  void loGFeatureEncode(const cv::Mat1b &src, unsigned int n_scale,
                        int min_wave_length, double mult, double sigma_onf,
                        cv::Mat1b &T, cv::Mat1b &M);

  int n_scale_;
  int min_wave_length_;
  float mult_;
  float sigma_onf_;
  int match_num_;

  flann::Index<flann::L2<float>> vec_list_;
  std::vector<FeatureDesc> feature_list_;
  std::vector<int> frame_index_list_;
  flann::Matrix<int> indices_;
  flann::Matrix<float> dists_;
};
}  // namespace cvte_lidar_slam

#endif