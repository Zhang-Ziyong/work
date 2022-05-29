#ifndef _POINT_CLOUD_H_
#define _POINT_CLOUD_H_

#include <vector>

#include "Eigen/Core"
#include "common/rigid_transform.hpp"
#include "glog/logging.h"
#include "time.hpp"

#include "proto/point_cloud.pb.h"

namespace slam2d_core {
namespace common {
struct RangePoint {
  Eigen::Vector3d position;
};

inline RangePoint operator*(const common::Rigid3 &lhs, const RangePoint &rhs) {
  RangePoint result = rhs;
  result.position = lhs * rhs.position;
  return result;
}

inline bool operator==(const RangePoint &lhs, const RangePoint &rhs) {
  return lhs.position == rhs.position;
}

class PointCloud {
 public:
  PointCloud();
  explicit PointCloud(std::vector<RangePoint> points);
  explicit PointCloud(const slam2d::mapping::proto::PointCloud &proto);

  bool empty() const;
  size_t size() const;
  void push_back(RangePoint value);
  const std::vector<RangePoint> &points() const;
  const RangePoint &operator[](const size_t index) const;

  slam2d::mapping::proto::PointCloud toProto() const;

  using ConstIterator = std::vector<RangePoint>::const_iterator;

  ConstIterator end() const;
  ConstIterator begin() const;

  template <class UnaryPredicate>
  PointCloud copy_if(UnaryPredicate predicate) const {
    std::vector<RangePoint> points;
    for (size_t index = 0; index < size(); ++index) {
      const RangePoint &point = points_[index];
      if (predicate(point)) {
        points.push_back(point);
      }
    }
    return PointCloud(points);
  }

 private:
  std::vector<RangePoint> points_;
};

struct TimedPointCloudData {
  Time time;
  Eigen::Vector3d origin;
  PointCloud ranges;
  std::vector<double> lengths;
};

struct RangeData {
  Eigen::Vector3d origin;
  PointCloud returns;
  PointCloud misses;
};

RangeData transformRangeData(const RangeData &range_data,
                             const common::Rigid3 &transform);

PointCloud transformPointCloud(const PointCloud &point_cloud,
                               const common::Rigid3 &transform);

}  // namespace common
}  // namespace slam2d_core

#endif
