#include "common/image.hpp"
namespace slam2d_core {
namespace common {

UniqueCairoSurfacePtr makeUniqueCairoSurfacePtr(cairo_surface_t *surface) {
  return UniqueCairoSurfacePtr(surface, cairo_surface_destroy);
}

UniqueCairoPtr makeUniqueCairoPtr(cairo_t *surface) {
  return UniqueCairoPtr(surface, cairo_destroy);
}

}  // namespace common
}  // namespace slam2d_core