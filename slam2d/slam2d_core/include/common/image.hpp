#ifndef _IMAGE_H_
#define _IMAGE_H_
#include "cairo/cairo.h"
#include <memory>

namespace slam2d_core {
namespace common {

constexpr cairo_format_t kCairoFormat = CAIRO_FORMAT_ARGB32;

using UniqueCairoSurfacePtr =
    std::unique_ptr<cairo_surface_t, void (*)(cairo_surface_t *)>;

UniqueCairoSurfacePtr makeUniqueCairoSurfacePtr(cairo_surface_t *surface);

using UniqueCairoPtr = std::unique_ptr<cairo_t, void (*)(cairo_t *)>;

UniqueCairoPtr makeUniqueCairoPtr(cairo_t *surface);

}  // namespace common
}  // namespace slam2d_core

#endif