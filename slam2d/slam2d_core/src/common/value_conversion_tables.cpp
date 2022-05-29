#include "common/value_conversion_tables.hpp"

#include "glog/logging.h"
#include <limits>
#include <map>
#include <memory>
#include <vector>

namespace slam2d_core {
namespace common {
namespace {

constexpr uint16_t kUpdateMarker = 1u << 15;

// 0 is unknown, [1, 32767] maps to [lower_bound, upper_bound].
float slowValueToBoundedFloat(const uint16_t value,
                              const uint16_t unknown_value,
                              const float unknown_result,
                              const float lower_bound,
                              const float upper_bound) {
  CHECK_LE(value, 32767);
  if (value == unknown_value)
    return unknown_result;
  const float kScale = (upper_bound - lower_bound) / 32766.f;
  return value * kScale + (lower_bound - kScale);
}

std::unique_ptr<std::vector<float>> precomputeValueToBoundedFloat(
    const uint16_t unknown_value, const float unknown_result,
    const float lower_bound, const float upper_bound) {
  auto result = std::make_unique<std::vector<float>>();
  size_t num_values = std::numeric_limits<uint16_t>::max() + 1;
  result->reserve(num_values);
  for (size_t value = 0; value != num_values; ++value) {
    result->push_back(slowValueToBoundedFloat(
        static_cast<uint16_t>(value) & ~kUpdateMarker, unknown_value,
        unknown_result, lower_bound, upper_bound));
  }
  return result;
}
}  // namespace

const std::vector<float> *ValueConversionTables::getConversionTable(
    float unknown_result, float lower_bound, float upper_bound) {
  std::tuple<float, float, float> bounds =
      std::make_tuple(unknown_result, lower_bound, upper_bound);
  auto lookup_table_iterator = bounds_to_lookup_table_.find(bounds);
  if (lookup_table_iterator == bounds_to_lookup_table_.end()) {
    auto insertion_result = bounds_to_lookup_table_.emplace(
        bounds, precomputeValueToBoundedFloat(0, unknown_result, lower_bound,
                                              upper_bound));
    return insertion_result.first->second.get();
  } else {
    return lookup_table_iterator->second.get();
  }
}

}  // namespace common
}  // namespace slam2d_core
