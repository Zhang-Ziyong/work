
#ifndef _COMMON_MATH_H_
#define _COMMON_MATH_H_

#include <cmath>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace slam2d_core {
namespace common {

template <typename T>
T clamp(const T value, const T min, const T max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

inline double degToRad(double deg) {
  return M_PI * deg / 180.;
}

inline double radToDeg(double rad) {
  return 180. * rad / M_PI;
}

inline int roundToInt(const float x) {
  return std::lround(x);
}

inline int roundToInt(const double x) {
  return std::lround(x);
}

inline int64_t roundToInt64(const float x) {
  return std::lround(x);
}

inline int64_t roundToInt64(const double x) {
  return std::lround(x);
}

inline double normalize_angle(double angle) {
  double a = fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
  if (a > M_PI)
    a -= 2.0 * M_PI;
  return a;
}

inline double quaternionToYaw(const Eigen::Quaternion<double> &quaternion) {
  double yaw = 0;
  double sqw = 0;
  double sqx = 0;
  double sqy = 0;
  double sqz = 0;
  sqx = quaternion.x() * quaternion.x();
  sqy = quaternion.y() * quaternion.y();
  sqz = quaternion.z() * quaternion.z();
  sqw = quaternion.w() * quaternion.w();

  double sarg =
      -2 * (quaternion.x() * quaternion.z() - quaternion.w() * quaternion.y()) /
      (sqx + sqy + sqz + sqw); /* normalization added from urdfom_headers */
  if (sarg <= -0.99999) {
    yaw = -2 * atan2(quaternion.y(), quaternion.x());
  } else if (sarg >= 0.99999) {
    yaw = 2 * atan2(quaternion.y(), quaternion.x());
  } else {
    yaw = atan2(
        2 * (quaternion.x() * quaternion.y() + quaternion.w() * quaternion.z()),
        sqw + sqx - sqy - sqz);
  }
  return yaw;
}

inline Eigen::Quaternion<double> yawToQuaternion(const double &roll,
                                                 const double &pitch,
                                                 const double &yaw) {
  double halfYaw = yaw * 0.5;
  double halfPitch = pitch * 0.5;
  double halfRoll = roll * 0.5;
  double cosYaw = std::cos(halfYaw);
  double sinYaw = std::sin(halfYaw);
  double cosPitch = std::cos(halfPitch);
  double sinPitch = std::sin(halfPitch);
  double cosRoll = std::cos(halfRoll);
  double sinRoll = std::sin(halfRoll);
  Eigen::Quaternion<double> q;
  q.x() = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
  q.y() = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
  q.z() = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
  q.w() = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;

  return q;
}

inline Eigen::Matrix<double, 3, 1> rotationQuaternionToAngleAxisVector(
    const Eigen::Quaternion<double> &quaternion) {
  Eigen::Quaternion<double> normalized_quaternion = quaternion.normalized();
  if (normalized_quaternion.w() < 0.) {
    normalized_quaternion.w() = -1. * normalized_quaternion.w();
    normalized_quaternion.x() = -1. * normalized_quaternion.x();
    normalized_quaternion.y() = -1. * normalized_quaternion.y();
    normalized_quaternion.z() = -1. * normalized_quaternion.z();
  }
  const double angle =
      2. * atan2(normalized_quaternion.vec().norm(), normalized_quaternion.w());
  constexpr double kCutoffAngle = 1e-7;  // We linearize below this angle.
  const double scale = angle < kCutoffAngle ? 2.0 : angle / sin(angle / 2.);
  return Eigen::Matrix<double, 3, 1>(scale * normalized_quaternion.x(),
                                     scale * normalized_quaternion.y(),
                                     scale * normalized_quaternion.z());
}

inline Eigen::Quaternion<double> angleAxisVectorToRotationQuaternion(
    const Eigen::Matrix<double, 3, 1> &angle_axis) {
  double scale = 0.5;
  double w = 1.0;
  constexpr double kCutoffAngle = 1e-8;  // We linearize below this angle.
  if (angle_axis.squaredNorm() > kCutoffAngle) {
    const double norm = angle_axis.norm();
    scale = sin(norm / 2.) / norm;
    w = cos(norm / 2.);
  }
  const Eigen::Matrix<double, 3, 1> quaternion_xyz = scale * angle_axis;
  return Eigen::Quaternion<double>(w, quaternion_xyz.x(), quaternion_xyz.y(),
                                   quaternion_xyz.z());
}

// Bring the 'difference' between two angles into [-pi; pi].
template <typename T>
T normalizeAngleDifference(T difference) {
  const T kPi = T(M_PI);
  while (difference > kPi) difference -= 2. * kPi;
  while (difference < -kPi) difference += 2. * kPi;
  return difference;
}

template <typename T>
constexpr T Power(T base, int exponent) {
  return (exponent != 0) ? base * Power(base, exponent - 1) : T(1);
}
template <typename T>
constexpr T Pow2(T a) {
  return Power(a, 2);
}

inline double angleDiff(double a, double b) {
  double d1, d2;
  a = normalize_angle(a);
  b = normalize_angle(b);
  d1 = a - b;
  d2 = 2 * M_PI - std::fabs(d1);
  if (d1 > 0) {
    d2 *= -1.0;
  }
  if (fabs(d1) < std::fabs(d2)) {
    return (d1);
  } else {
    return (d2);
  }
}

}  // namespace common
}  // namespace slam2d_core

#endif
