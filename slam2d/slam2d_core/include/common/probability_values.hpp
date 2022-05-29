#ifndef _PROBABILITY_VALUES_H_
#define _PROBABILITY_VALUES_H_

#include "glog/logging.h"
#include "math.hpp"
#include <cmath>
#include <vector>

namespace slam2d_core {
namespace common {

namespace {

inline uint16_t boundedFloatToValue(const float float_value,
                                    const float lower_bound,
                                    const float upper_bound) {
  const int value =
      common::roundToInt(
          (common::clamp(float_value, lower_bound, upper_bound) - lower_bound) *
          (32766.f / (upper_bound - lower_bound))) +
      1;
  // DCHECK for performance.
  DCHECK_GE(value, 1);
  DCHECK_LE(value, 32767);
  return value;
}

}  // namespace

// odd模型表示概率
inline float Odds(float probability) {
  return probability / (1.f - probability);
}

// odd模型转换成概率
inline float probabilityFromOdds(const float odds) {
  return odds / (odds + 1.f);
}

// hit概率转换成free概率
inline float probabilityToCorrespondenceCost(const float probability) {
  return 1.f - probability;
}

// free概率转换成hit概率
inline float correspondenceCostToProbability(const float correspondence_cost) {
  return 1.f - correspondence_cost;
}

// 概率空间范围限制，默认0.1~0.9
constexpr float kMinProbability = 0.1f;                   // 最小占用概率
constexpr float kMaxProbability = 1.f - kMinProbability;  // 最大占用概率
constexpr float kMinCorrespondenceCost = 1.f - kMaxProbability;  // 最小free概率
constexpr float kMaxCorrespondenceCost = 1.f - kMinProbability;  // 最大free概率

// 判断是否在占用概率区间，超过最大最小放回最大最小，其余放回原本值
inline float clampProbability(const float probability) {
  return common::clamp(probability, kMinProbability, kMaxProbability);
}

// 判断是否在free概率区间，超过最大最小放回最大最小，其余放回原本值
inline float clampCorrespondenceCost(const float correspondence_cost) {
  return common::clamp(correspondence_cost, kMinCorrespondenceCost,
                       kMaxCorrespondenceCost);
}

constexpr uint16_t kUnknownProbabilityValue = 0;
constexpr uint16_t kUnknownCorrespondenceValue = kUnknownProbabilityValue;
// 为避免重复更新，采用+kUpdateMarker作为当前range是否被更新的标志位Marker，并同时记录在update_index中【方便后期遍历，减少消耗】。
constexpr uint16_t kUpdateMarker = 1u << 15;

// 将free概率转成1~32767的数
inline uint16_t correspondenceCostToValue(const float correspondence_cost) {
  return boundedFloatToValue(correspondence_cost, kMinCorrespondenceCost,
                             kMaxCorrespondenceCost);
}

// 将占用概率转成1~32767的数
inline uint16_t probabilityToValue(const float probability) {
  return boundedFloatToValue(probability, kMinProbability, kMaxProbability);
}

extern const std::vector<float> *const kValueToProbability;
extern const std::vector<float> *const kValueToCorrespondenceCost;

// 获取1~32767的数对应的占用概率
inline float valueToProbability(const uint16_t value) {
  return (*kValueToProbability)[value];
}

// 获取1~32767的数对应的free概率
inline float valueToCorrespondenceCost(const uint16_t value) {
  return (*kValueToCorrespondenceCost)[value];
}

// 占用概率值转换成free概率值
inline uint16_t probabilityValueToCorrespondenceCostValue(
    uint16_t probability_value) {
  if (probability_value == kUnknownProbabilityValue) {
    return kUnknownCorrespondenceValue;
  }
  bool update_carry = false;
  if (probability_value > kUpdateMarker) {
    probability_value -= kUpdateMarker;
    update_carry = true;
  }
  uint16_t result = correspondenceCostToValue(
      probabilityToCorrespondenceCost(valueToProbability(probability_value)));
  if (update_carry)
    result += kUpdateMarker;
  return result;
}

// free占用概率值转换成占用概率值
inline uint16_t correspondenceCostValueToProbabilityValue(
    uint16_t correspondence_cost_value) {
  if (correspondence_cost_value == kUnknownCorrespondenceValue)
    return kUnknownProbabilityValue;
  bool update_carry = false;
  if (correspondence_cost_value > kUpdateMarker) {
    correspondence_cost_value -= kUpdateMarker;
    update_carry = true;
  }
  uint16_t result = probabilityToValue(correspondenceCostToProbability(
      valueToCorrespondenceCost(correspondence_cost_value)));
  if (update_carry)
    result += kUpdateMarker;
  return result;
}

std::vector<uint16_t> computeLookupTableToApplyOdds(float odds);
std::vector<uint16_t> computeLookupTableToApplyCorrespondenceCostOdds(
    float odds);

}  // namespace common
}  // namespace slam2d_core

#endif
