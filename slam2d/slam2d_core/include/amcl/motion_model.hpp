#ifndef _MOTION_MODEL_H_
#define _MOTION_MODEL_H_
#include "sample.hpp"
#include "common/math.hpp"
#include "common/rigid_transform.hpp"

#include <vector>
#include <map>
#include <Eigen/Core>

namespace slam2d_core {
namespace amcl {

/**
 * HistogramIndex
 * @brief 用于统计粒子分布直方图的直方图索引，对应状态
 **/
class HistogramIndex {
 public:
  int index_[3];  ///< 对应状态统计直方图索引
  /**
   * operator==
   * @brief 判断两个索引是否相同
   * @param[in] 比较的索引
   * @return true-相同， false-不同
   **/
  bool operator==(const HistogramIndex &cell) const {
    if (this->index_[0] != cell.index_[0] ||
        this->index_[1] != cell.index_[1] ||
        this->index_[2] != cell.index_[2]) {
      return false;
    } else {
      return true;
    }
  }

  /**
   * operator<
   * @brief 比较两个索引大小，用于map
   * @param[in] 比较的索引
   * @return true-小于， false-不小于
   **/
  bool operator<(const HistogramIndex &cell) const {
    if (this->index_[0] != cell.index_[0]) {
      return this->index_[0] < cell.index_[0];
    } else if (this->index_[1] != cell.index_[1]) {
      return this->index_[1] < cell.index_[1];
    } else if (this->index_[2] != cell.index_[2]) {
      return this->index_[2] < cell.index_[2];
    } else {
      return false;
    }
  }
};

class MotionModel {
 public:
  MotionModel() = default;
  ~MotionModel() = default;

  MotionModel(const MotionModel &) = delete;
  MotionModel &operator=(const MotionModel &) = delete;

  bool updateAction(std::vector<Sample> &samples,
                    const unsigned int sample_count,
                    const common::Rigid3 &new_system_status,
                    const common::Rigid3 &last_system_status);

  bool computeStatus(const std::vector<Sample> &samples,
                     const unsigned int sample_count,
                     Eigen::Vector3d &pose_mean, Eigen::Vector3d &pose_cov);

  bool setInitStatus(std::vector<Sample> &samaple,
                     const unsigned int sample_count,
                     const Eigen::Vector3d &mean, const Eigen::Vector3d &cov);

  inline unsigned int getHistogramCount() { return histogram_.size(); }

  void addHistogramSample(const common::Rigid3 &sample_pose);
  inline void clearHistogram() { histogram_.clear(); }
  void setMotionMode(const double alpha1, const double alpha2,
                     const double alpha3, const double alpha4,
                     const double alpha5) {
    alpha1_ = alpha1;
    alpha2_ = alpha2;
    alpha3_ = alpha3;
    alpha4_ = alpha4;
    alpha5_ = alpha5;
    init_ = true;
  }
  void setHistogramCellSize(const double x, const double y, const double yaw);

 private:
  double randomGaussian(double sigma);

  double alpha1_;      ///< 模型参数
  double alpha2_;      ///< 模型参数
  double alpha3_;      ///< 模型参数
  double alpha4_;      ///< 模型参数
  double alpha5_;      ///< 模型参数
  bool init_ = false;  ///< 判断当前是否已初始化

  double cell_size_x_ = 0.5;    ///< 统计直方图分辨率
  double cell_size_y_ = 0.5;    ///< 统计直方图分辨率
  double cell_size_yaw_ = 0.5;  ///< 统计直方图分辨率

  std::map<HistogramIndex, unsigned int>
      histogram_;  ///< 用于统计粒子集分布情况的直方图
};

}  // namespace amcl
}  // namespace slam2d_core

#endif