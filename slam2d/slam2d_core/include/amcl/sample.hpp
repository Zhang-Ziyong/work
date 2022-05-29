#ifndef _SAMPLE_H_
#define _SAMPLE_H_

#include "common/rigid_transform.hpp"

namespace slam2d_core {
namespace amcl {
/**
 * Sample
 * @brief 粒子滤波的粒子
 **/
class Sample {
 public:
  double weight = 0.0;
  common::Rigid3 pose;
};

/**
 * PFSamples
 * @brief 粒子滤波的粒子集
 **/
class PFSamples {
 public:
  double pop_z;              ///< KLD采样参数
  double pop_err;            ///< KLD采样参数
  double w_slow;             ///< 重采样参数，用于生成随机粒子
  double w_fast;             ///< 重采样参数，用于生成随机粒子
  double alpha_slow;         ///< KLD采样参数
  double alpha_fast;         ///< KLD采样参数
  bool converged[2];         ///< 粒子是否收敛
  double dist_treshold;      ///< 粒子最大距离阈值
  unsigned int min_samples;  ///< 最小粒子数
  unsigned int max_samples;  ///< 最大粒子数

  unsigned int current_set;      ///< 当前粒子集
  std::vector<Sample> set[2];    ///< 粒子集,两个粒子集
  unsigned int sample_count[2];  ///< 有效粒子数
};

}  // namespace amcl
}  // namespace slam2d_core

#endif