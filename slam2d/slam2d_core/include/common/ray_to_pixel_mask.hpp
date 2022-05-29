#ifndef _RAY_TO_PIXEL_MASK_H_
#define _RAY_TO_PIXEL_MASK_H_

#include "glog/logging.h"
#include "math.hpp"
#include <vector>

namespace slam2d_core {
namespace common {

std::vector<Eigen::Array2i> rayToPixelMask(const Eigen::Array2i &scaled_begin,
                                           const Eigen::Array2i &scaled_end,
                                           int subpixel_scale);

}  // namespace common
}  // namespace slam2d_core

#endif
