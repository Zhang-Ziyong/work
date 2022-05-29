/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "line_segment2d.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "math_utils.h"

namespace cvte {
namespace hdmap {
// constexpr double kMathEpsilon = 1e-10;

bool IsWithin(double val, double bound1, double bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }
  return val >= bound1 - math_utils::kMathEpsilon && val <= bound2 + math_utils::kMathEpsilon;
}

LineSegment2d::LineSegment2d() { unit_direction_ = Vec2d(1, 0); }

LineSegment2d::LineSegment2d(const Vec2d &start, const Vec2d &end)
    : start_(start), end_(end) {
  const double dx = end_.x() - start_.x();
  const double dy = end_.y() - start_.y();
  length_ = hypot(dx, dy);
  unit_direction_ =
      (length_ <= math_utils::kMathEpsilon ? Vec2d(0, 0)
                               : Vec2d(dx / length_, dy / length_));
  heading_ = atan2(dy, dx);
}

Vec2d LineSegment2d::rotate(const double angle) {
  Vec2d diff_vec = end_ - start_;
  Eigen::Matrix<double, 2, 2> rotate;
  rotate << cos(angle), -sin(angle), sin(angle), cos(angle);
  diff_vec = rotate * diff_vec;
  return start_ + diff_vec;
}

double LineSegment2d::length() const { return length_; }

double LineSegment2d::length_sqr() const { return length_ * length_; }

double LineSegment2d::DistanceTo(const Vec2d &point) const {
  if (length_ <= math_utils::kMathEpsilon) {
    return (point - start_).norm();
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    return hypot(x0, y0);
  }
  if (proj >= length_) {
    return (point - end_).norm();
  }
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double LineSegment2d::DistanceTo(const Vec2d &point,
                                 Vec2d *const nearest_pt) const {
  // CHECK_NOTNULL(nearest_pt);
  if (length_ <= math_utils::kMathEpsilon) {
    *nearest_pt = start_;
    return (point - start_).norm();
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj < 0.0) {
    *nearest_pt = start_;
    return hypot(x0, y0);
  }
  if (proj > length_) {
    *nearest_pt = end_;
    return (point - end_).norm();
  }
  *nearest_pt = start_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double LineSegment2d::DistanceSquareTo(const Vec2d &point) const {
  if (length_ <= math_utils::kMathEpsilon) {
    return (point - start_).squaredNorm();
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    return math_utils::Square(x0) + math_utils::Square(y0);
  }
  if (proj >= length_) {
    return (point - end_).squaredNorm();
  }
  return math_utils::Square(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double LineSegment2d::DistanceSquareTo(const Vec2d &point,
                                       Vec2d *const nearest_pt) const {
  // CHECK_NOTNULL(nearest_pt);
  if (length_ <= math_utils::kMathEpsilon) {
    *nearest_pt = start_;
    return (point - start_).squaredNorm();
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    *nearest_pt = start_;
    return math_utils::Square(x0) + math_utils::Square(y0);
  }
  if (proj >= length_) {
    *nearest_pt = end_;
    return (point - end_).squaredNorm();
  }
  *nearest_pt = start_ + unit_direction_ * proj;
  return math_utils::Square(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

bool LineSegment2d::IsPointIn(const Vec2d &point) const {
  if (length_ <= math_utils::kMathEpsilon) {
    return std::abs(point.x() - start_.x()) <= math_utils::kMathEpsilon &&
           std::abs(point.y() - start_.y()) <= math_utils::kMathEpsilon;
  }
  const double prod = math_utils::CrossProd(point, start_, end_);
  if (std::abs(prod) > math_utils::kMathEpsilon) {
    return false;
  }
  return IsWithin(point.x(), start_.x(), end_.x()) &&
         IsWithin(point.y(), start_.y(), end_.y());
}

double LineSegment2d::ProjectOntoUnit(const Vec2d &point) const {
  return unit_direction_.transpose() * (point - start_);
}

double LineSegment2d::ProductOntoUnit(const Vec2d &point) const {
  return math_utils::CrossProd(unit_direction_.x(), unit_direction_.y(),
    (point - start_).x(), (point - start_).y());
}

bool LineSegment2d::HasIntersect(const LineSegment2d &other_segment) const {
  Vec2d point;
  return GetIntersect(other_segment, &point);
}

bool LineSegment2d::GetIntersect(const LineSegment2d &other_segment,
                                 Vec2d *const point) const {
  // CHECK_NOTNULL(point);
  if (IsPointIn(other_segment.start())) {
    *point = other_segment.start();
    return true;
  }
  if (IsPointIn(other_segment.end())) {
    *point = other_segment.end();
    return true;
  }
  if (other_segment.IsPointIn(start_)) {
    *point = start_;
    return true;
  }
  if (other_segment.IsPointIn(end_)) {
    *point = end_;
    return true;
  }
  if (length_ <= math_utils::kMathEpsilon || other_segment.length() <= math_utils::kMathEpsilon) {
    return false;
  }
  const double cc1 = math_utils::CrossProd(start_, end_, other_segment.start());
  const double cc2 = math_utils::CrossProd(start_, end_, other_segment.end());
  if (cc1 * cc2 >= -math_utils::kMathEpsilon) {
    return false;
  }
  const double cc3 =
      math_utils::CrossProd(other_segment.start(), other_segment.end(), start_);
  const double cc4 =
      math_utils::CrossProd(other_segment.start(), other_segment.end(), end_);
  if (cc3 * cc4 >= -math_utils::kMathEpsilon) {
    return false;
  }
  const double ratio = cc4 / (cc4 - cc3);
  *point = Vec2d(start_.x() * ratio + end_.x() * (1.0 - ratio),
                 start_.y() * ratio + end_.y() * (1.0 - ratio));
  return true;
}

// return distance with perpendicular foot point.
double LineSegment2d::GetPerpendicularFoot(const Vec2d &point,
                                           Vec2d *const foot_point) const {
  // CHECK_NOTNULL(foot_point);
  if (length_ <= math_utils::kMathEpsilon) {
    *foot_point = start_;
    return (point - start_).norm();
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  *foot_point = start_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}
}
}  // namespace math
