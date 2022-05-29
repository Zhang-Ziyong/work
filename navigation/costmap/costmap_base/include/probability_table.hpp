#ifndef PROBABILITY_TABLE_HPP
#define PROBABILITY_TABLE_HPP
#include <cmath>
#include <vector>
namespace CVTE_BABOT {
class ProbabilityTable {
 private:
  std::vector<float> kValueToProbability_;
  std::vector<unsigned int> hit_table_;
  std::vector<unsigned int> miss_table_;
  double hit_probability_;
  double miss_probability_;

  const unsigned int kUnknownProbabilityValue_ = 0;

  // 概率值的范围为 [0.1, 0.9]
  const float kMinProbability_ = 0.1f;
  const float kMaxProbability_ = 1.f - kMinProbability_;

  const float kScale_ = (kMaxProbability_ - kMinProbability_) / 32766.f;

  void update() {}

  inline float calculateOdds(float probability) {
    return probability / (1.f - probability);
  }

  inline float probabilityFromOdds(const float odds) {
    return odds / (odds + 1.f);
  }

  inline float clampProbability(const float probability) {
    return clamp(probability, kMinProbability_, kMaxProbability_);
  }

  inline float clamp(const float value, const float min, const float max) {
    if (value > max) {
      return max;
    }
    if (value < min) {
      return min;
    }
    return value;
  }

  float calculateValueToProbability(const unsigned int value);

  void precomputeValueToProbability();
  std::vector<unsigned int> computeLookupTableToApplyOdds(const float odds);
  unsigned int updateValue(const unsigned int cell,
                           const std::vector<unsigned int>& table);

 public:
  ProbabilityTable(const double hit_probability, const double miss_probability);
  ~ProbabilityTable();

  inline unsigned int probabilityToValue(const float probability) {
    const int value =
        std::lround((clampProbability(probability) - kMinProbability_) *
                    (32766.f / (kMaxProbability_ - kMinProbability_))) +
        1;
    // DCHECK for performance.
    // DCHECK_GE(value, 1);
    // DCHECK_LE(value, 32767);
    return value;
  }
  inline float valueToProbability(const unsigned int value) {
    return kValueToProbability_[value];
  }
  unsigned int hit(const unsigned int cell);
  unsigned int miss(const unsigned int cell);
  unsigned int unknow(const unsigned int cell);
  const unsigned int kUpdateMarker_ = 1u << 15;  // = 32768
};

}  // namespace CVTE_BABOT

#endif