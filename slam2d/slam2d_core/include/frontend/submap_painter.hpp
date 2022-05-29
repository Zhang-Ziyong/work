#ifndef _SUBMAP_PAINTER_H_
#define _SUBMAP_PAINTER_H_

#include "common/image.hpp"
#include "common/rigid_transform.hpp"
#include <map>
#include <vector>
namespace slam2d_core {
namespace frontend {

struct PaintSubmapSlicesResult {
  PaintSubmapSlicesResult(common::UniqueCairoSurfacePtr surface,
                          Eigen::Array2f origin)
      : surface(std::move(surface)), origin(origin) {}
  common::UniqueCairoSurfacePtr surface;
  Eigen::Array2f origin;
};

struct SubmapSlice {
  SubmapSlice() : surface(common::makeUniqueCairoSurfacePtr(nullptr)) {}

  int width;
  int height;
  int version;
  double resolution;
  common::Rigid3 slice_pose;
  common::UniqueCairoSurfacePtr surface;
  std::vector<uint32_t> cairo_data;

  common::Rigid3 pose;
  int metadata_version = -1;
};

PaintSubmapSlicesResult paintSubmapSlices(
    const std::map<int, SubmapSlice> &submaps,
    double resolution);  // int -> submap id.

common::UniqueCairoSurfacePtr drawTexture(const std::vector<char> &intensity,
                                          const std::vector<char> &alpha,
                                          int width, int height,
                                          std::vector<uint32_t> *cairo_data);

}  // namespace frontend
}  // namespace slam2d_core

#endif