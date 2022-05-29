#include "frontend/submap_painter.hpp"
namespace slam2d_core {
namespace frontend {

namespace {

Eigen::Affine3d toEigen(const common::Rigid3 &rigid3) {
  return Eigen::Translation3d(rigid3.translation()) * rigid3.rotation();
}

void cairoPaintSubmapSlices(
    const double scale, const std::map<int, SubmapSlice> &submaps, cairo_t *cr,
    std::function<void(const SubmapSlice &)> draw_callback) {
  cairo_scale(cr, scale, scale);

  for (auto &pair : submaps) {
    const auto &submap_slice = pair.second;
    if (submap_slice.surface == nullptr) {
      LOG(WARNING) << "submap_slice.surface is nullptr.";
      return;
    }

    const Eigen::Matrix4d homo =
        toEigen(submap_slice.pose * submap_slice.slice_pose).matrix();

    cairo_save(cr);
    cairo_matrix_t matrix;
    cairo_matrix_init(&matrix, homo(1, 0), homo(0, 0), -homo(1, 1), -homo(0, 1),
                      homo(0, 3), -homo(1, 3));
    cairo_transform(cr, &matrix);

    const double submap_resolution = submap_slice.resolution;
    cairo_scale(cr, submap_resolution, submap_resolution);

    // Invokes caller's callback to utilize slice data in global cooridnate
    // frame. e.g. finds bounding box, paints slices.
    draw_callback(submap_slice);
    cairo_restore(cr);
  }
}

}  // namespace

PaintSubmapSlicesResult paintSubmapSlices(
    const std::map<int, SubmapSlice> &submaps, const double resolution) {
  Eigen::AlignedBox2f bounding_box;
  {
    auto surface = common::makeUniqueCairoSurfacePtr(
        cairo_image_surface_create(common::kCairoFormat, 1, 1));
    auto cr = common::makeUniqueCairoPtr(cairo_create(surface.get()));
    const auto update_bounding_box = [&bounding_box, &cr](double x, double y) {
      cairo_user_to_device(cr.get(), &x, &y);
      bounding_box.extend(Eigen::Vector2f(x, y));
    };

    cairoPaintSubmapSlices(
        1. / resolution, submaps, cr.get(),
        [&update_bounding_box](const SubmapSlice &submap_slice) {
          update_bounding_box(0, 0);
          update_bounding_box(submap_slice.width, 0);
          update_bounding_box(0, submap_slice.height);
          update_bounding_box(submap_slice.width, submap_slice.height);
        });
  }

  const int kPaddingPixel = 5;
  const Eigen::Array2i size(
      std::ceil(bounding_box.sizes().x()) + 2 * kPaddingPixel,
      std::ceil(bounding_box.sizes().y()) + 2 * kPaddingPixel);
  const Eigen::Array2f origin(-bounding_box.min().x() + kPaddingPixel,
                              -bounding_box.min().y() + kPaddingPixel);

  auto surface = common::makeUniqueCairoSurfacePtr(
      cairo_image_surface_create(common::kCairoFormat, size.x(), size.y()));
  {
    auto cr = common::makeUniqueCairoPtr(cairo_create(surface.get()));
    cairo_set_source_rgba(cr.get(), 0.5, 0.0, 0.0, 1.);
    cairo_paint(cr.get());
    cairo_translate(cr.get(), origin.x(), origin.y());
    cairoPaintSubmapSlices(1. / resolution, submaps, cr.get(),
                           [&cr](const SubmapSlice &submap_slice) {
                             cairo_set_source_surface(
                                 cr.get(), submap_slice.surface.get(), 0., 0.);
                             cairo_paint(cr.get());
                           });
    cairo_surface_flush(surface.get());
  }
  return PaintSubmapSlicesResult(std::move(surface), origin);
}

common::UniqueCairoSurfacePtr drawTexture(
    const std::vector<char> &intensity, const std::vector<char> &alpha,
    const int width, const int height,
    std::vector<uint32_t> *const cairo_data) {
  CHECK(cairo_data->empty());

  const int expected_stride = 4 * width;
  CHECK_EQ(expected_stride,
           cairo_format_stride_for_width(common::kCairoFormat, width));
  for (size_t i = 0; i < intensity.size(); ++i) {
    const uint8_t intensity_value = intensity.at(i);
    const uint8_t alpha_value = alpha.at(i);
    const uint8_t observed =
        (intensity_value == 0 && alpha_value == 0) ? 0 : 255;
    cairo_data->push_back((alpha_value << 24) | (intensity_value << 16) |
                          (observed << 8) | 0);
  }

  auto surface =
      common::makeUniqueCairoSurfacePtr(cairo_image_surface_create_for_data(
          reinterpret_cast<unsigned char *>(cairo_data->data()),
          common::kCairoFormat, width, height, expected_stride));
  CHECK_EQ(cairo_surface_status(surface.get()), CAIRO_STATUS_SUCCESS)
      << cairo_status_to_string(cairo_surface_status(surface.get()));
  return surface;
}

}  // namespace frontend
}  // namespace slam2d_core