#include "common/ray_to_pixel_mask.hpp"

#include "Eigen/Dense"

namespace slam2d_core {
namespace common {
namespace {

bool isEqual(const Eigen::Array2i &lhs, const Eigen::Array2i &rhs) {
  return ((lhs - rhs).matrix().lpNorm<1>() == 0);
}
}  // namespace

std::vector<Eigen::Array2i> rayToPixelMask(const Eigen::Array2i &scaled_begin,
                                           const Eigen::Array2i &scaled_end,
                                           int subpixel_scale) {
  if (scaled_begin.x() > scaled_end.x()) {
    return rayToPixelMask(scaled_end, scaled_begin, subpixel_scale);
  }

  CHECK_GE(scaled_begin.x(), 0);
  CHECK_GE(scaled_begin.y(), 0);
  CHECK_GE(scaled_end.y(), 0);
  std::vector<Eigen::Array2i> pixel_mask;

  if (scaled_begin.x() / subpixel_scale == scaled_end.x() / subpixel_scale) {
    Eigen::Array2i current(
        scaled_begin.x() / subpixel_scale,
        std::min(scaled_begin.y(), scaled_end.y()) / subpixel_scale);
    pixel_mask.push_back(current);
    const int end_y =
        std::max(scaled_begin.y(), scaled_end.y()) / subpixel_scale;
    for (; current.y() <= end_y; ++current.y()) {
      if (!isEqual(pixel_mask.back(), current))
        pixel_mask.push_back(current);
    }
    return pixel_mask;
  }

  const int64_t dx = scaled_end.x() - scaled_begin.x();
  const int64_t dy = scaled_end.y() - scaled_begin.y();
  const int64_t denominator = 2 * subpixel_scale * dx;

  Eigen::Array2i current = scaled_begin / subpixel_scale;
  pixel_mask.push_back(current);

  int64_t sub_y = (2 * (scaled_begin.y() % subpixel_scale) + 1) * dx;

  const int first_pixel =
      2 * subpixel_scale - 2 * (scaled_begin.x() % subpixel_scale) - 1;

  const int last_pixel = 2 * (scaled_end.x() % subpixel_scale) + 1;

  const int end_x = std::max(scaled_begin.x(), scaled_end.x()) / subpixel_scale;

  sub_y += dy * first_pixel;
  if (dy > 0) {
    while (true) {
      if (!isEqual(pixel_mask.back(), current))
        pixel_mask.push_back(current);
      while (sub_y > denominator) {
        sub_y -= denominator;
        ++current.y();
        if (!isEqual(pixel_mask.back(), current))
          pixel_mask.push_back(current);
      }
      ++current.x();
      if (sub_y == denominator) {
        sub_y -= denominator;
        ++current.y();
      }
      if (current.x() == end_x) {
        break;
      }

      sub_y += dy * 2 * subpixel_scale;
    }

    sub_y += dy * last_pixel;
    if (!isEqual(pixel_mask.back(), current))
      pixel_mask.push_back(current);
    while (sub_y > denominator) {
      sub_y -= denominator;
      ++current.y();
      if (!isEqual(pixel_mask.back(), current))
        pixel_mask.push_back(current);
    }
    CHECK_NE(sub_y, denominator);
    CHECK_EQ(current.y(), scaled_end.y() / subpixel_scale);
    return pixel_mask;
  }

  while (true) {
    if (!isEqual(pixel_mask.back(), current))
      pixel_mask.push_back(current);
    while (sub_y < 0) {
      sub_y += denominator;
      --current.y();
      if (!isEqual(pixel_mask.back(), current))
        pixel_mask.push_back(current);
    }
    ++current.x();
    if (sub_y == 0) {
      sub_y += denominator;
      --current.y();
    }
    if (current.x() == end_x) {
      break;
    }
    sub_y += dy * 2 * subpixel_scale;
  }
  sub_y += dy * last_pixel;
  if (!isEqual(pixel_mask.back(), current))
    pixel_mask.push_back(current);
  while (sub_y < 0) {
    sub_y += denominator;
    --current.y();
    if (!isEqual(pixel_mask.back(), current))
      pixel_mask.push_back(current);
  }
  CHECK_NE(sub_y, 0);
  CHECK_EQ(current.y(), scaled_end.y() / subpixel_scale);
  return pixel_mask;
}

}  // namespace common
}  // namespace slam2d_core
