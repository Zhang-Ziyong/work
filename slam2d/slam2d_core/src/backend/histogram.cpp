#include "backend/histogram.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <numeric>
#include <string>

#include "glog/logging.h"

namespace slam2d_core {
namespace backend {

void Histogram::add(const float value) {
  values_.push_back(value);
}

std::string Histogram::toString(const int buckets) const {
  CHECK_GE(buckets, 1);
  if (values_.empty()) {
    return "Count: 0";
  }
  const float min = *std::min_element(values_.begin(), values_.end());
  const float max = *std::max_element(values_.begin(), values_.end());
  const float mean =
      std::accumulate(values_.begin(), values_.end(), 0.f) / values_.size();
  std::to_string(values_.size());

  std::string result = "Count: " + std::to_string(values_.size()) +
                       "  Min: " + std::to_string(min) +
                       "  Max: " + std::to_string(max) +
                       "  Mean: " + std::to_string(mean);

  if (min == max) {
    return result;
  }

  CHECK_LT(min, max);
  float lower_bound = min;
  int total_count = 0;
  for (int i = 0; i != buckets; ++i) {
    const float upper_bound =
        (i + 1 == buckets)
            ? max
            : (max * (i + 1) / buckets + min * (buckets - i - 1) / buckets);
    int count = 0;
    for (const float value : values_) {
      if (lower_bound <= value &&
          (i + 1 == buckets ? value <= upper_bound : value < upper_bound)) {
        ++count;
      }
    }
    total_count += count;
    result += "\n[" + std::to_string(lower_bound) + ", " +
              std::to_string(upper_bound) + (i + 1 == buckets ? ']' : ')');

    constexpr int kMaxBarChars = 20;
    const int bar =
        (count * kMaxBarChars + values_.size() / 2) / values_.size();
    result += "\t";
    for (int i = 0; i != kMaxBarChars; ++i) {
      result += (i < (kMaxBarChars - bar)) ? " " : "#";
    }

    result += "\tCount: " + std::to_string(count) + " (" +
              std::to_string(count * 1e2f / values_.size()) +
              "%)\tTotal: " + std::to_string(total_count) + " (" +
              std::to_string(total_count * 1e2f / values_.size()) + "%)";
    lower_bound = upper_bound;
  }
  return result;
}

}  // namespace backend
}  // namespace slam2d_core
