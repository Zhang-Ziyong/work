#include "common/point_cloud.hpp"

namespace slam2d_core {
namespace common {

PointCloud::PointCloud() {}

PointCloud::PointCloud(std::vector<RangePoint> points)
    : points_(std::move(points)) {}

PointCloud::PointCloud(const slam2d::mapping::proto::PointCloud &proto) {
  const int data_size = proto.point_data_size();
  points_.reserve(data_size);

  for (int i = 0; i != data_size; ++i) {
    RangePoint point;
    point.position = common::toEigen(proto.point_data(i));
    points_.emplace_back(point);
  }
}

size_t PointCloud::size() const {
  return points_.size();
}
bool PointCloud::empty() const {
  return points_.empty();
}

const std::vector<RangePoint> &PointCloud::points() const {
  return points_;
}

const RangePoint &PointCloud::operator[](const size_t index) const {
  return points_[index];
}

PointCloud::ConstIterator PointCloud::begin() const {
  return points_.begin();
}
PointCloud::ConstIterator PointCloud::end() const {
  return points_.end();
}

void PointCloud::push_back(RangePoint value) {
  points_.push_back(std::move(value));
}

slam2d::mapping::proto::PointCloud PointCloud::toProto() const {
  slam2d::mapping::proto::PointCloud proto;
  proto.mutable_point_data()->Reserve(points_.size());
  for (const auto point : points_) {
    *proto.add_point_data() = common::toProto(point.position);
  }
  return proto;
}

PointCloud transformPointCloud(const PointCloud &point_cloud,
                               const common::Rigid3 &transform) {
  std::vector<RangePoint> points;
  points.reserve(point_cloud.size());
  for (const RangePoint &point : point_cloud.points()) {
    points.emplace_back(transform * point);
  }
  return PointCloud(points);
}

RangeData transformRangeData(const RangeData &range_data,
                             const common::Rigid3 &transform) {
  return RangeData{
      transform * range_data.origin,
      transformPointCloud(range_data.returns, transform),
      transformPointCloud(range_data.misses, transform),
  };
}

}  // namespace common
}  // namespace slam2d_core
