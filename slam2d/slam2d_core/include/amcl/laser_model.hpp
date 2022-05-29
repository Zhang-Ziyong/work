#ifndef _LASER_MODEL_H_
#define _LASER_MODEL_H_
#include <vector>

#include "common/point_cloud.hpp"
#include "common/rigid_transform.hpp"
#include "common/time.hpp"
#include "occupancy_grid.hpp"
#include "sample.hpp"

namespace slam2d_core {
namespace amcl {

class LaserScanData {
 public:
  LaserScanData() {}
  /**
   * LaserScanData
   * @brief 构造函数
   * @param[in] count-一帧计算数据点数
   **/
  LaserScanData(const unsigned int count) {
    range_count = count;
    ranges.resize(count);
    ranges_angle.resize(count);
  }

  LaserScanData DownSample(float down_sample_dist) const {
    LaserScanData scan;
    scan.time = this->time;
    scan.range_max = this->range_max;
    scan.range_min = this->range_min;
    if (this->ranges.size() < 2) {
      scan.ranges = this->ranges;
      scan.ranges_angle = this->ranges_angle;
    } else {
      for (size_t i = 0; i < this->ranges.size() - 1; i++) {
        if (scan.range_min < this->ranges[i] &&
            this->ranges[i] < scan.range_max && scan.ranges.size() == 0) {
          scan.ranges.push_back(this->ranges[i]);
          scan.ranges_angle.push_back(this->ranges_angle[i]);
        }

        if (scan.ranges.size() > 0 && scan.range_min < this->ranges[i + 1] &&
            this->ranges[i + 1] < scan.range_max) {
          float x1 = scan.ranges.back() * std::cos(scan.ranges_angle.back());
          float y1 = scan.ranges.back() * std::sin(scan.ranges_angle.back());
          float x2 = this->ranges[i + 1] * std::cos(this->ranges_angle[i + 1]);
          float y2 = this->ranges[i + 1] * std::sin(this->ranges_angle[i + 1]);
          //相邻点距离大于给定降采样距离则选取插入
          if ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) >
              down_sample_dist * down_sample_dist) {
            scan.ranges.push_back(this->ranges[i + 1]);
            scan.ranges_angle.push_back(this->ranges_angle[i + 1]);
          }
        }
      }
    }
    return scan;
  }

  unsigned int range_count;  // 一帧计算数据点数
  double range_max;          // 最大有效测量距离
  double range_min;          // 最小有效测量距离
  common::Time time;         // 数据时间

  std::vector<double> ranges;        // 测量距离
  std::vector<double> ranges_angle;  // 测量角度
};

slam2d_core::common::PointCloud LaserScanDataToPointCloud(
    const LaserScanData &ld);

class LaserModel {
 public:
  LaserModel() = default;
  LaserModel(const LaserModel &obj) = delete;
  LaserModel &operator=(const LaserModel &obj) = delete;

  double updateSensor(std::vector<Sample> &samples,
                      const unsigned int sample_count,
                      const LaserScanData &sensor_data);

  double likelihoodFieldModel(std::vector<Sample> &samples,
                              const unsigned int sample_count,
                              const LaserScanData &sensor_data);

  void setMap(const std::shared_ptr<OccupancyGrid> ptr_occupancy_map) {
    ptr_occupancy_map_ = ptr_occupancy_map;
  }

  inline void setLaserTF(const common::Rigid3 &laser_tf) {
    laser_tf_ = laser_tf;
  }

  void setModelLikelihoodField(const double z_hit, const double z_rand,
                               const double sigma_hit,
                               const double max_occ_dist, const int max_beams) {
    z_hit_ = z_hit;
    z_rand_ = z_rand;
    sigma_hit_ = sigma_hit;
    max_occ_dist_ = max_occ_dist;
    max_beams_ = max_beams;
  }

 private:
  double z_hit_;         ///< 观察模型算法参数
  double z_rand_;        ///< 观察模型算法参数
  int max_beams_;        ///< 最大的匹配激光点数
  double sigma_hit_;     ///< 观察模型算法参数
  double max_occ_dist_;  ///< 观察模型算法参数，最大比较距离

  common::Rigid3 laser_tf_;  ///< 激光->base坐标
  std::shared_ptr<OccupancyGrid> ptr_occupancy_map_ = nullptr;
  //     RBNN rbnn_;                              ///<　RBNN聚类器
  //   std::vector<rbnn_point_t> rbnn_points_;  ///< 聚类数据缓存
};

}  // namespace amcl
}  // namespace slam2d_core

#endif