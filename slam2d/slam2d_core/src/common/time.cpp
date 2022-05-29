#include "common/time.hpp"
#include <cerrno>
#include <cstring>
#include <string>
#include <time.h>

#include "glog/logging.h"

namespace slam2d_core {
namespace common {

Duration fromSeconds(const double seconds) {
  return std::chrono::duration_cast<Duration>(
      std::chrono::duration<double>(seconds));
}

double toSeconds(const Duration duration) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration)
      .count();
}

double toSeconds(const std::chrono::steady_clock::duration duration) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration)
      .count();
}

Time fromUniversal(const int64_t ticks) {
  return Time(Duration(ticks));
}

int64_t toUniversal(const Time time) {
  return time.time_since_epoch().count();
}

Duration fromMilliseconds(const int64_t milliseconds) {
  return std::chrono::duration_cast<Duration>(
      std::chrono::milliseconds(milliseconds));
}

std::ostream &operator<<(std::ostream &os, const Time time) {
  os << std::to_string(toUniversal(time));
  return os;
}

}  // namespace common

}  // namespace slam2d_core
