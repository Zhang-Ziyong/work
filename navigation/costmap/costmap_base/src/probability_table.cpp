#include "probability_table.hpp"
#include <glog/logging.h>
#include <iostream>
namespace CVTE_BABOT {
ProbabilityTable::ProbabilityTable(const double hit_probability,
                                   const double miss_probability)
    : hit_probability_(hit_probability), miss_probability_(miss_probability) {
  precomputeValueToProbability();
  hit_table_ = computeLookupTableToApplyOdds(calculateOdds(hit_probability_));
  miss_table_ = computeLookupTableToApplyOdds(calculateOdds(miss_probability_));
  // std::cout << "0.5 = " << probabilityToValue(0.5) << std::endl;
}

ProbabilityTable::~ProbabilityTable() {}

float ProbabilityTable::calculateValueToProbability(const unsigned int value) {
  if (value == kUnknownProbabilityValue_) {
    // Unknown cells have kMinProbability.
    return kMinProbability_;
  }

  return value * kScale_ + (kMinProbability_ - kScale_);
}

void ProbabilityTable::precomputeValueToProbability() {
  for (int value = 0; value != 32768; ++value) {
    kValueToProbability_.push_back(calculateValueToProbability(value));
  }
}
std::vector<unsigned int> ProbabilityTable::computeLookupTableToApplyOdds(
    const float odds) {
  std::vector<unsigned int> result;
  // 计算Value=0时更新后的Value，可直接更新，因为更新前=0为Unknown
  result.push_back(probabilityToValue(probabilityFromOdds(odds)));
  // 从第2个开始，到32767
  for (int cell = 1; cell != 32768; ++cell) {
    result.push_back(probabilityToValue(
        probabilityFromOdds(odds * calculateOdds(kValueToProbability_[cell]))));
  }
  return result;
}

unsigned int ProbabilityTable::hit(const unsigned int cell) {
  return updateValue(cell, hit_table_);
}
unsigned int ProbabilityTable::miss(const unsigned int cell) {
  return updateValue(cell, miss_table_);
}
unsigned int ProbabilityTable::unknow(const unsigned int cell) {
  return cell + kUpdateMarker_;
}

unsigned int ProbabilityTable::updateValue(
    const unsigned int cell, const std::vector<unsigned int>& table) {
  return table[cell];
}

}  // namespace CVTE_BABOT