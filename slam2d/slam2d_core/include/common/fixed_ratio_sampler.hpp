#ifndef _FIXED_RATIO_SAMPLER_H_
#define _FIXED_RATIO_SAMPLER_H_

#include <string>

namespace slam2d_core {
namespace common {

class FixedRatioSampler {
 public:
  explicit FixedRatioSampler(double ratio);
  ~FixedRatioSampler();

  FixedRatioSampler(const FixedRatioSampler &) = delete;
  FixedRatioSampler &operator=(const FixedRatioSampler &) = delete;

  bool Pulse();

  std::string DebugString();

 private:
  const double ratio_;

  int64_t num_pulses_ = 0;
  int64_t num_samples_ = 0;
};

}  // namespace common
}  // namespace slam2d_core

#endif
