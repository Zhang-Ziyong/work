#ifndef _HISTOGRAM_H_
#define _HISTOGRAM_H_

#include <string>
#include <vector>

namespace slam2d_core {
namespace backend {

class Histogram {
 public:
  void add(float value);
  std::string toString(int buckets) const;

 private:
  std::vector<float> values_;
};

}  // namespace backend
}  // namespace slam2d_core

#endif
