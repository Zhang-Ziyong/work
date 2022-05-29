#include "common/probability_values.hpp"
#include <memory>

namespace slam2d_core {
namespace common {

namespace {

constexpr int kValueCount = 32768;

// 0 is unknown, [1, 32767] maps to [lower_bound, upper_bound].
float slowValueToBoundedFloat(const uint16_t value,
                              const uint16_t unknown_value,
                              const float unknown_result,
                              const float lower_bound,
                              const float upper_bound) {
  CHECK_LT(value, kValueCount);
  if (value == unknown_value)
    return unknown_result;
  const float kScale = (upper_bound - lower_bound) / (kValueCount - 2.f);
  return value * kScale + (lower_bound - kScale);
}

// 提前计算所有uint16中的对应有边界范围的float数，即存储到表格中
std::unique_ptr<std::vector<float>> precomputeValueToBoundedFloat(
    const uint16_t unknown_value, const float unknown_result,
    const float lower_bound, const float upper_bound) {
  auto result = std::make_unique<std::vector<float>>();
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a probability.
  constexpr int kRepetitionCount = 2;
  result->reserve(kRepetitionCount * kValueCount);
  for (int repeat = 0; repeat != kRepetitionCount; ++repeat) {
    for (int value = 0; value != kValueCount; ++value) {
      result->push_back(slowValueToBoundedFloat(
          value, unknown_value, unknown_result, lower_bound, upper_bound));
    }
  }
  return result;
}

std::unique_ptr<std::vector<float>> precomputeValueToProbability() {
  return precomputeValueToBoundedFloat(kUnknownProbabilityValue,
                                       kMinProbability, kMinProbability,
                                       kMaxProbability);
}

std::unique_ptr<std::vector<float>> precomputeValueToCorrespondenceCost() {
  return precomputeValueToBoundedFloat(
      kUnknownCorrespondenceValue, kMaxCorrespondenceCost,
      kMinCorrespondenceCost, kMaxCorrespondenceCost);
}

}  // namespace

const std::vector<float> *const kValueToProbability =
    precomputeValueToProbability().release();

const std::vector<float> *const kValueToCorrespondenceCost =
    precomputeValueToCorrespondenceCost().release();

std::vector<uint16_t> computeLookupTableToApplyOdds(const float odds) {
  std::vector<uint16_t> result;
  result.reserve(kValueCount);
  result.push_back(probabilityToValue(probabilityFromOdds(odds)) +
                   kUpdateMarker);
  for (int cell = 1; cell != kValueCount; ++cell) {
    result.push_back(probabilityToValue(probabilityFromOdds(
                         odds * Odds((*kValueToProbability)[cell]))) +
                     kUpdateMarker);
  }
  return result;
}

// 计算free的概率表
std::vector<uint16_t> computeLookupTableToApplyCorrespondenceCostOdds(
    float odds) {
  std::vector<uint16_t> result;
  result.reserve(kValueCount);
  result.push_back(correspondenceCostToValue(probabilityToCorrespondenceCost(
                       probabilityFromOdds(odds))) +
                   kUpdateMarker);
  // 针对每一个value均乘以更新系数odds
  for (int cell = 1; cell != kValueCount; ++cell) {
    result.push_back(
        correspondenceCostToValue(
            probabilityToCorrespondenceCost(probabilityFromOdds(
                odds * Odds(correspondenceCostToProbability(
                           (*kValueToCorrespondenceCost)[cell]))))) +
        kUpdateMarker);
  }
  return result;
}

}  // namespace common
}  // namespace slam2d_core
