#include "common/rigid_transform.hpp"

namespace slam2d_core {
namespace common {

// Conversions between Eigen and proto.
Rigid2 toRigid2(const slam2d::mapping::proto::Rigid2 &transform) {
  return Rigid2({transform.translation().x(), transform.translation().y()},
                transform.rotation());
}

Eigen::Vector2d toEigen(const slam2d::mapping::proto::Vector2d &vector) {
  return Eigen::Vector2d(vector.x(), vector.y());
}

Eigen::Vector3d toEigen(const slam2d::mapping::proto::Vector3d &vector) {
  return Eigen::Vector3d(vector.x(), vector.y(), vector.z());
}

Eigen::Quaterniond toEigen(
    const slam2d::mapping::proto::Quaterniond &quaternion) {
  return Eigen::Quaterniond(quaternion.w(), quaternion.x(), quaternion.y(),
                            quaternion.z());
}

slam2d::mapping::proto::Rigid2 toProto(const Rigid2 &transform) {
  slam2d::mapping::proto::Rigid2 proto;
  proto.mutable_translation()->set_x(transform.translation().x());
  proto.mutable_translation()->set_y(transform.translation().y());
  proto.set_rotation(transform.rotation().angle());
  return proto;
}

slam2d::mapping::proto::Rigid3 toProto(const Rigid3 &rigid) {
  slam2d::mapping::proto::Rigid3 proto;
  *proto.mutable_translation() = toProto(rigid.translation());
  *proto.mutable_rotation() = toProto(rigid.rotation());
  return proto;
}

Rigid3 toRigid3(const slam2d::mapping::proto::Rigid3 &rigid) {
  return Rigid3(toEigen(rigid.translation()), toEigen(rigid.rotation()));
}

slam2d::mapping::proto::Vector2d toProto(const Eigen::Vector2d &vector) {
  slam2d::mapping::proto::Vector2d proto;
  proto.set_x(vector.x());
  proto.set_y(vector.y());
  return proto;
}

slam2d::mapping::proto::Vector3d toProto(const Eigen::Vector3d &vector) {
  slam2d::mapping::proto::Vector3d proto;
  proto.set_x(vector.x());
  proto.set_y(vector.y());
  proto.set_z(vector.z());
  return proto;
}

slam2d::mapping::proto::Quaterniond toProto(
    const Eigen::Quaterniond &quaternion) {
  slam2d::mapping::proto::Quaterniond proto;
  proto.set_w(quaternion.w());
  proto.set_x(quaternion.x());
  proto.set_y(quaternion.y());
  proto.set_z(quaternion.z());
  return proto;
}

}  // namespace common
}  // namespace slam2d_core