#ifndef _PARTICLE_FILTER_H_
#define _PARTICLE_FILTER_H_

#include <map>
#include <vector>

#include "common/rigid_transform.hpp"
#include "laser_model.hpp"
#include "motion_model.hpp"
#include "sample.hpp"

namespace slam2d_core {
namespace amcl {

/**
 * ParticleFilterOptions
 * @brief 粒子滤波相关参数设置
 **/
class ParticleFilterOptions {
 public:
  double pop_z = 1;            ///< KLD参数
  double pop_err = 0.01;       ///< KLD参数
  double dist_treshold = 0.5;  ///< 粒子最大的距离

  double alpha_slow = 0.0;  ///< 生成随机粒子参数
  double alpha_fast = 0.0;  ///< 生成随机粒子参数

  unsigned int min_samples = 1;  ///< 最小粒子数
  unsigned int max_samples = 1;  ///< 最大粒子数
};

class ParticleFilter {
 public:
  explicit ParticleFilter(const ParticleFilterOptions &options);
  ParticleFilter(const ParticleFilter &obj) = delete;
  ParticleFilter &operator=(const ParticleFilter &obj) = delete;
  ~ParticleFilter();

  bool updateAction(MotionModel &action_model,
                    const common::Rigid3 &action_data,
                    const common::Rigid3 &last_system_statu);

  bool updateSensor(LaserModel &laser_mode, const LaserScanData &sensor_data);

  bool updateResample(MotionModel &model);

  void getCurrentPFSet(std::vector<Sample> &current_set);

  bool computeStatus(MotionModel &model, Eigen::Vector3d &mean,
                     Eigen::Vector3d &cov);

  bool setInitStatus(MotionModel &model, const Eigen::Vector3d &mean,
                     const Eigen::Vector3d &cov);
  void clear();

  inline std::vector<Sample> &getSamples() {
    return samples_.set[samples_.current_set % 2];
  }

  inline unsigned int getSamplesCount() {
    return samples_.sample_count[samples_.current_set % 2];
  }

  void setOptions(const ParticleFilterOptions &options);

 private:
  void initialization();
  unsigned int resampleLimit(const unsigned int k);

  bool init_ = false;  ///< 用于判断当前是否完成初始化

  PFSamples samples_;              ///< 粒子集
  ParticleFilterOptions options_;  ///< 算法参数

  std::vector<double> probalility_table_;  ///< 概率表，用于重采样
  std::map<unsigned int, unsigned int> resample_limit_map_;
};

}  // namespace amcl
}  // namespace slam2d_core
#endif