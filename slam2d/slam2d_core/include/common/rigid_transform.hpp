#ifndef _COMMON_RIGID_H_
#define _COMMON_RIGID_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "glog/logging.h"
#include "math.hpp"
#include "time.hpp"
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "proto/transform.pb.h"

namespace slam2d_core {
namespace common {

class Rigid2 {
 public:
  using Vector = Eigen::Vector2d;
  using Rotation2D = Eigen::Rotation2Dd;

  Rigid2() : translation_(Vector::Zero()), rotation_(Rotation2D::Identity()) {}

  Rigid2(const Vector &translation, const Rotation2D &rotation)
      : translation_(translation), rotation_(rotation) {}

  Rigid2(const Vector &translation, const double rotation)
      : translation_(translation), rotation_(rotation) {}

  static Rigid2 Rotation(const double rotation) {
    return Rigid2(Vector::Zero(), rotation);
  }

  static Rigid2 Rotation(const Rotation2D &rotation) {
    return Rigid2(Vector::Zero(), rotation);
  }

  static Rigid2 Translation(const Vector &vector) {
    return Rigid2(vector, Rotation2D::Identity());
  }

  static Rigid2 Identity() { return Rigid2(); }

  //作用是什么，是否一定需要
  Rigid2 cast() const {
    return Rigid2(translation_.template cast<double>(),
                  rotation_.template cast<double>());
  }

  const Vector &translation() const { return translation_; }

  Rotation2D rotation() const { return rotation_; }

  double normalizedAngle() const {
    return common::normalize_angle(rotation().angle());
  }

  Rigid2 inverse() const {
    const Rotation2D rotation = rotation_.inverse();
    const Vector translation = -(rotation * translation_);
    return Rigid2(translation, rotation);
  }

 private:
  Vector translation_;
  Rotation2D rotation_;
};

inline Rigid2 operator*(const Rigid2 &lhs, const Rigid2 &rhs) {
  return Rigid2(lhs.rotation() * rhs.translation() + lhs.translation(),
                lhs.rotation() * rhs.rotation());
}

inline Rigid2::Vector operator*(const Rigid2 &rigid,
                                const Rigid2::Vector &point) {
  return rigid.rotation() * point + rigid.translation();
}

class Rigid3 {
 public:
  using Vector = Eigen::Matrix<double, 3, 1>;
  using Quaternion = Eigen::Quaternion<double>;
  using AngleAxis = Eigen::AngleAxis<double>;

  Rigid3() : translation_(Vector::Zero()), rotation_(Quaternion::Identity()) {}
  Rigid3(const Vector &translation, const Quaternion &rotation)
      : translation_(translation), rotation_(rotation) {}
  Rigid3(const Vector &translation, const AngleAxis &rotation)
      : translation_(translation), rotation_(rotation) {}

  static Rigid3 Rotation(const AngleAxis &angle_axis) {
    return Rigid3(Vector::Zero(), Quaternion(angle_axis));
  }

  static Rigid3 Rotation(const Quaternion &rotation) {
    return Rigid3(Vector::Zero(), rotation);
  }

  static Rigid3 Translation(const Vector &vector) {
    return Rigid3(vector, Quaternion::Identity());
  }

  static Rigid3 FromArrays(const std::array<double, 4> &rotation,
                           const std::array<double, 3> &translation) {
    return Rigid3(Eigen::Map<const Vector>(translation.data()),
                  Eigen::Quaternion<double>(rotation[0], rotation[1],
                                            rotation[2], rotation[3]));
  }

  static Rigid3 Identity() { return Rigid3(); }

  Rigid3 cast() const {
    return Rigid3(translation_.template cast<double>(),
                  rotation_.template cast<double>());
  }

  const Vector &translation() const { return translation_; }
  const Quaternion &rotation() const { return rotation_; }

  double getAngle() {
    return double(2) *
           std::atan2(rotation_.vec().norm(), std::abs(rotation_.w()));
  }

  Rigid3 inverse() const {
    const Quaternion rotation = rotation_.conjugate();
    const Vector translation = -(rotation * translation_);
    return Rigid3(translation, rotation);
  }

  bool IsValid() const {
    return !std::isnan(translation_.x()) && !std::isnan(translation_.y()) &&
           !std::isnan(translation_.z()) &&
           std::abs(1.0 - rotation_.norm()) < double(1e-3);
  }

 private:
  Vector translation_;
  Quaternion rotation_;
};

inline Rigid3 operator*(const Rigid3 &lhs, const Rigid3 &rhs) {
  return Rigid3(lhs.rotation() * rhs.translation() + lhs.translation(),
                (lhs.rotation() * rhs.rotation()).normalized());
}

inline Rigid3::Vector operator*(const Rigid3 &rigid,
                                const typename Rigid3::Vector &point) {
  return rigid.rotation() * point + rigid.translation();
}

inline double getYaw(const Eigen::Quaterniond &rotation) {
  const Eigen::Matrix<double, 3, 1> direction =
      rotation * Eigen::Matrix<double, 3, 1>::UnitX();
  return atan2(direction.y(), direction.x());
}

inline double getYaw(const Rigid3 &transform) {
  return getYaw(transform.rotation());
}

inline Rigid2 project2D(const Rigid3 &transform) {
  return Rigid2(transform.translation().template head<2>(), getYaw(transform));
}

inline Rigid3 embed3D(const Rigid2 &transform) {
  return Rigid3({transform.translation().x(), transform.translation().y(), 0.0},
                Eigen::AngleAxis<double>(transform.rotation().angle(),
                                         Eigen::Matrix<double, 3, 1>::UnitZ()));
}

struct TimestampedTransform {
  Time time;
  Rigid3 transform;
};

inline TimestampedTransform interpolate(const TimestampedTransform &start,
                                        const TimestampedTransform &end,
                                        const Time time) {
  CHECK_LE(start.time, time);
  CHECK_GE(end.time, time);

  const double duration = common::toSeconds(end.time - start.time);
  const double factor = common::toSeconds(time - start.time) / duration;
  const Eigen::Vector3d origin =
      start.transform.translation() +
      (end.transform.translation() - start.transform.translation()) * factor;
  const Eigen::Quaterniond rotation =
      Eigen::Quaterniond(start.transform.rotation())
          .slerp(factor, Eigen::Quaterniond(end.transform.rotation()));
  return TimestampedTransform{time, common::Rigid3(origin, rotation)};
}

// Conversions between Eigen and proto.
Rigid2 toRigid2(const slam2d::mapping::proto::Rigid2 &transform);
Eigen::Vector2d toEigen(const slam2d::mapping::proto::Vector2d &vector);
Eigen::Vector3d toEigen(const slam2d::mapping::proto::Vector3d &vector);
Eigen::Quaterniond toEigen(
    const slam2d::mapping::proto::Quaterniond &quaternion);
slam2d::mapping::proto::Rigid2 toProto(const Rigid2 &transform);
slam2d::mapping::proto::Rigid3 toProto(const Rigid3 &rigid);
Rigid3 toRigid3(const slam2d::mapping::proto::Rigid3 &rigid);
slam2d::mapping::proto::Vector2d toProto(const Eigen::Vector2d &vector);
slam2d::mapping::proto::Vector3d toProto(const Eigen::Vector3d &vector);
slam2d::mapping::proto::Quaterniond toProto(
    const Eigen::Quaterniond &quaternion);

}  // namespace common
}  // namespace slam2d_core

#endif
