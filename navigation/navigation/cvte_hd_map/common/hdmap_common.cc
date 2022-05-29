#include <algorithm>
#include <limits>
#include "glog/logging.h"
#include "hdmap_common.hpp"
#include "math_utils.h"
#include "hdmap_impl.hpp"

const double FLAGS_half_vehicle_width = 0.5;

namespace cvte {
namespace hdmap {
namespace {

// Minimum error in lane segmentation.
// const double kSegmentationEpsilon = 0.2;

// Minimum distance to remove duplicated points.
constexpr double kDuplicatedPointsEpsilon = 1e-7;

// Margin for comparation
constexpr double kEpsilon = 0.1;

/**
 * @brief create a Map ID given a string.
 * @param id a string id
 * @return a Map ID instance
 */
inline cvte::hdmap::Id MakeMapId(const std::string &id) {
  cvte::hdmap::Id map_id;
  map_id.set_id(id);
  return map_id;
}

void RemoveDuplicates(std::vector<Vec2d> *points) {
  if (points == nullptr) {
    return;
  }

  int count = 0;
  const double limit = kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
  for (const auto &point : *points) {
    if (count == 0 || (point - (*points)[count - 1]).norm() > limit) {
      (*points)[count++] = point;
    }
  }
  points->resize(count);
}

void PointsFromCurve(const cvte::hdmap::Curve &input_curve,
                     std::vector<Vec2d> *points) {
  if (points == nullptr) {
    return;
  }
  points->clear();

  for (const auto &curve : input_curve.segment()) {
    if (curve.has_line_segment()) {
      for (const auto &point : curve.line_segment().point()) {
        points->emplace_back(point.x(), point.y());
      }
    } else {
      LOG(ERROR) << "Can not handle curve type.";
    }
  }
  RemoveDuplicates(points);
}

Polygon2d ConvertToPolygon2d(const cvte::hdmap::Polygon &polygon) {
  std::vector<Vec2d> points;
  points.reserve(polygon.point_size());
  for (const auto &point : polygon.point()) {
    points.emplace_back(point.x(), point.y());
  }
  RemoveDuplicates(&points);
  while (points.size() >= 2 &&
         (points[0] - points.back()).norm() <= math_utils::kMathEpsilon) {
    points.pop_back();
  }
  return Polygon2d(points);
}

void SegmentsFromCurve(const cvte::hdmap::Curve &curve,
                       std::vector<LineSegment2d> *segments) {
  if (segments == nullptr) {
    return;
  }

  std::vector<Vec2d> points;
  PointsFromCurve(curve, &points);
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    segments->emplace_back(points[i], points[i + 1]);
  }
}

double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t) {
  if (std::abs(t1 - t0) <= math_utils::kMathEpsilon) {
    LOG(INFO) << "input time difference is too small";
    return math_utils::NormalizeAngle(a0);
  }
  const double a0_n = math_utils::NormalizeAngle(a0);
  const double a1_n = math_utils::NormalizeAngle(a1);
  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;
  }

  const double r = (t - t0) / (t1 - t0);
  const double a = a0_n + d * r;
  return math_utils::NormalizeAngle(a);
}

}  // namespace

LaneInfo::LaneInfo(const cvte::hdmap::Lane &lane) : lane_(lane) {
  Init();
}

void LaneInfo::Init() {
  PointsFromCurve(lane_.central_curve(), &points_);
  CHECK_GE(points_.size(), 2U);
  segments_.clear();
  accumulated_s_.clear();
  unit_directions_.clear();
  headings_.clear();

  double s = 0;
  for (size_t i = 0; i + 1 < points_.size(); ++i) {
    segments_.emplace_back(points_[i], points_[i + 1]);
    accumulated_s_.push_back(s);
    unit_directions_.push_back(segments_.back().unit_direction());
    s += segments_.back().length();
  }

  accumulated_s_.push_back(s);
  total_length_ = s;
  // ACHECK(!unit_directions_.empty());
  unit_directions_.push_back(unit_directions_.back());
  for (const auto &direction : unit_directions_) {
    headings_.push_back(atan2(direction(1), direction(0)));
  }
  for (const auto &overlap_id : lane_.overlap_id()) {
    overlap_ids_.emplace_back(overlap_id.id());
  }
  // ACHECK(!segments_.empty());

  sampled_left_width_.clear();
  sampled_right_width_.clear();
  for (const auto &sample : lane_.left_sample()) {
    sampled_left_width_.emplace_back(sample.s(), sample.width());
  }
  for (const auto &sample : lane_.right_sample()) {
    sampled_right_width_.emplace_back(sample.s(), sample.width());
  }

  if (lane_.has_type()) {
    if (lane_.type() == Lane::CITY_DRIVING) {
      for (const auto &p : sampled_left_width_) {
        if (p.second < FLAGS_half_vehicle_width) {
          LOG(ERROR)
              << "lane[id = " << lane_.id().DebugString()
              << "]. sampled_left_width_[" << p.second
              << "] is too small. It should be larger than half vehicle width["
              << FLAGS_half_vehicle_width << "].";
        }
      }
      for (const auto &p : sampled_right_width_) {
        if (p.second < FLAGS_half_vehicle_width) {
          LOG(ERROR)
              << "lane[id = " << lane_.id().DebugString()
              << "]. sampled_right_width_[" << p.second
              << "] is too small. It should be larger than half vehicle width["
              << FLAGS_half_vehicle_width << "].";
        }
      }
    } else if (lane_.type() == Lane::NONE) {
      LOG(ERROR) << "lane_[id = " << lane_.id().DebugString()
                 << "] type is NONE.";
    }
  } else {
    LOG(ERROR) << "lane_[id = " << lane_.id().DebugString() << "] has NO type.";
  }

  sampled_left_road_width_.clear();
  sampled_right_road_width_.clear();
  for (const auto &sample : lane_.left_road_sample()) {
    sampled_left_road_width_.emplace_back(sample.s(), sample.width());
  }
  for (const auto &sample : lane_.right_road_sample()) {
    sampled_right_road_width_.emplace_back(sample.s(), sample.width());
  }

  CreateKDTree();
}

void LaneInfo::GetWidth(const double s, double *left_width,
                        double *right_width) const {
  if (left_width != nullptr) {
    *left_width = GetWidthFromSample(sampled_left_width_, s);
  }
  if (right_width != nullptr) {
    *right_width = GetWidthFromSample(sampled_right_width_, s);
  }
}

double LaneInfo::Heading(const double s) const {
  const double kEpsilon = 0.001;
  if (s + kEpsilon < accumulated_s_.front()) {
    LOG(ERROR) << "s:" << s << " should be >= " << accumulated_s_.front();
    return 0.0;
  }
  if (s - kEpsilon > accumulated_s_.back()) {
    LOG(ERROR) << "s:" << s << " should be <= " << accumulated_s_.back();
    return 0.0;
  }

  auto iter = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  int index = static_cast<int>(std::distance(accumulated_s_.begin(), iter));
  if (index == 0 || *iter - s <= math_utils::kMathEpsilon) {
    return headings_[index];
  }
  return slerp(headings_[index - 1], accumulated_s_[index - 1],
               headings_[index], accumulated_s_[index], s);
}

double LaneInfo::Curvature(const double s) const {
  if (points_.size() < 2U) {
    LOG(ERROR) << "Not enough points to compute curvature.";
    return 0.0;
  }
  const double kEpsilon = 0.001;
  if (s + kEpsilon < accumulated_s_.front()) {
    LOG(ERROR) << "s:" << s << " should be >= " << accumulated_s_.front();
    return 0.0;
  }
  if (s > accumulated_s_.back() + kEpsilon) {
    LOG(ERROR) << "s:" << s << " should be <= " << accumulated_s_.back();
    return 0.0;
  }

  auto iter = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  if (iter == accumulated_s_.end()) {
    LOG(INFO) << "Reach the end of lane.";
    return 0.0;
  }
  int index = static_cast<int>(std::distance(accumulated_s_.begin(), iter));
  if (index == 0) {
    LOG(INFO) << "Reach the beginning of lane";
    return 0.0;
  }
  return (headings_[index] - headings_[index - 1]) /
         (accumulated_s_[index] - accumulated_s_[index - 1] + kEpsilon);
}

double LaneInfo::GetWidth(const double s) const {
  double left_width = 0.0;
  double right_width = 0.0;
  GetWidth(s, &left_width, &right_width);
  return left_width + right_width;
}

double LaneInfo::GetEffectiveWidth(const double s) const {
  double left_width = 0.0;
  double right_width = 0.0;
  GetWidth(s, &left_width, &right_width);
  return 2 * std::min(left_width, right_width);
}

void LaneInfo::GetRoadWidth(const double s, double *left_width,
                            double *right_width) const {
  if (left_width != nullptr) {
    *left_width = GetWidthFromSample(sampled_left_road_width_, s);
  }
  if (right_width != nullptr) {
    *right_width = GetWidthFromSample(sampled_right_road_width_, s);
  }
}

double LaneInfo::GetRoadWidth(const double s) const {
  double left_width = 0.0;
  double right_width = 0.0;
  GetRoadWidth(s, &left_width, &right_width);
  return left_width + right_width;
}

double LaneInfo::GetWidthFromSample(
    const std::vector<LaneInfo::SampledWidth> &samples, const double s) const {
  if (samples.empty()) {
    return 0.0;
  }
  if (s <= samples[0].first) {
    return samples[0].second;
  }
  if (s >= samples.back().first) {
    return samples.back().second;
  }
  int low = 0;
  int high = static_cast<int>(samples.size());
  while (low + 1 < high) {
    const int mid = (low + high) / 2;
    if (samples[mid].first <= s) {
      low = mid;
    } else {
      high = mid;
    }
  }
  const LaneInfo::SampledWidth &sample1 = samples[low];
  const LaneInfo::SampledWidth &sample2 = samples[high];
  const double ratio = (sample2.first - s) / (sample2.first - sample1.first);
  return sample1.second * ratio + sample2.second * (1.0 - ratio);
}

bool LaneInfo::IsOnLane(const Vec2d &point) const {
  double accumulate_s = 0.0;
  double lateral = 0.0;
  if (!GetProjection(point, &accumulate_s, &lateral)) {
    return false;
  }

  if (accumulate_s > (total_length() + kEpsilon) ||
      (accumulate_s + kEpsilon) < 0.0) {
    return false;
  }

  double left_width = 0.0;
  double right_width = 0.0;
  GetWidth(accumulate_s, &left_width, &right_width);
  if (lateral < left_width && lateral > -right_width) {
    return true;
  }
  return false;
}

bool LaneInfo::IsOnLane(const Box2d &box) const {
  std::vector<Vec2d> corners;
  box.GetAllCorners(&corners);
  for (const auto &corner : corners) {
    if (!IsOnLane(corner)) {
      return false;
    }
  }
  return true;
}

Vec2d LaneInfo::GetSmoothPoint(double s) const {
  // RETURN_VAL_IF(points_.size() < 2, point);
  if (points_.size() < 2) {
    return points_[0];
  }
  if (s <= 0.0) {
    return points_[0];
  }

  if (s >= total_length()) {
    return points_.back();
  }

  const auto low_itr =
      std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  // RETURN_VAL_IF(low_itr == accumulated_s_.end(), point);
  if (low_itr == accumulated_s_.end()) {
    return points_.back();
  }
  size_t index = low_itr - accumulated_s_.begin();
  double delta_s = *low_itr - s;
  if (delta_s < math_utils::kMathEpsilon) {
    return (points_[index]);
  }

  auto smooth_point = points_[index] - unit_directions_[index - 1] * delta_s;

  return smooth_point;
}

double LaneInfo::DistanceTo(const Vec2d &point) const {
  const auto segment_box = lane_segment_kdtree_->GetNearestObject(point);
  // RETURN_VAL_IF_NULL(segment_box, 0.0);
  if (segment_box == nullptr) {
    return 0.0;
  }
  return segment_box->DistanceTo(point);
}

double LaneInfo::DistanceTo(const Vec2d &point, Vec2d *map_point,
                            double *s_offset, int *s_offset_index) const {
  if (map_point == nullptr) {
    return 0.0;
  }
  if (s_offset == nullptr) {
    return 0.0;
  }
  if (s_offset_index == nullptr) {
    return 0.0;
  }
  // RETURN_VAL_IF_NULL(map_point, 0.0);
  // RETURN_VAL_IF_NULL(s_offset, 0.0);
  // RETURN_VAL_IF_NULL(s_offset_index, 0.0);
  if (map_point == nullptr || s_offset == nullptr ||
      s_offset_index == nullptr) {
    return 0.0;
  }

  const auto segment_box = lane_segment_kdtree_->GetNearestObject(point);
  // RETURN_VAL_IF_NULL(segment_box, 0.0);
  if (segment_box == nullptr) {
    return 0.0;
  }
  int index = segment_box->id();
  double distance = segments_[index].DistanceTo(point, map_point);
  *s_offset_index = index;
  *s_offset =
      accumulated_s_[index] + (segments_[index].start() - (*map_point)).norm();
  return distance;
}

Vec2d LaneInfo::GetNearestPoint(const Vec2d &point, double *distance) const {
  const auto segment_box = lane_segment_kdtree_->GetNearestObject(point);
  int index = segment_box->id();
  Vec2d nearest_point;
  *distance = segments_[index].DistanceTo(point, &nearest_point);

  return nearest_point;
}

bool LaneInfo::GetProjection(const Vec2d &point, double *accumulate_s,
                             double *lateral) const {
  // RETURN_VAL_IF_NULL(accumulate_s, false);
  // RETURN_VAL_IF_NULL(lateral, false);
  if (accumulate_s == nullptr || lateral == nullptr) {
    LOG(ERROR) << "accumulate_s or lateral is nullptr";
    return false;
  }

  if (segments_.empty()) {
    return false;
  }
  double min_dist = std::numeric_limits<double>::infinity();
  int seg_num = static_cast<int>(segments_.size());
  int min_index = 0;
  for (int i = 0; i < seg_num; ++i) {
    const double distance = segments_[i].DistanceSquareTo(point);
    if (distance < min_dist) {
      min_index = i;
      min_dist = distance;
    }
  }
  min_dist = std::sqrt(min_dist);
  const auto &nearest_seg = segments_[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  if (min_index == 0) {
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
    }
  } else if (min_index == seg_num - 1) {
    *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
    }
  } else {
    *accumulate_s = accumulated_s_[min_index] +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
  }
  return true;
}

void LaneInfo::PostProcess(const HdMapImpl &map_instance) {
  UpdateOverlaps(map_instance);
}

void LaneInfo::UpdateOverlaps(const HdMapImpl &map_instance) {
  for (const auto &overlap_id : overlap_ids_) {
    const auto &overlap_ptr =
        map_instance.GetOverlapById(MakeMapId(overlap_id));
    if (overlap_ptr == nullptr) {
      continue;
    }
    overlaps_.emplace_back(overlap_ptr);
    for (const auto &object : overlap_ptr->overlap().object()) {
      const auto &object_id = object.id().id();
      if (object_id == lane_.id().id()) {
        continue;
      }
      const auto &object_map_id = MakeMapId(object_id);
      if (map_instance.GetLaneById(object_map_id) != nullptr) {
        cross_lanes_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetSignalById(object_map_id) != nullptr) {
        signals_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetYieldSignById(object_map_id) != nullptr) {
        yield_signs_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetStopSignById(object_map_id) != nullptr) {
        stop_signs_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetCrosswalkById(object_map_id) != nullptr) {
        crosswalks_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetJunctionById(object_map_id) != nullptr) {
        junctions_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetClearAreaById(object_map_id) != nullptr) {
        clear_areas_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetSpeedBumpById(object_map_id) != nullptr) {
        speed_bumps_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetParkingSpaceById(object_map_id) != nullptr) {
        parking_spaces_.emplace_back(overlap_ptr);
      }
      if (map_instance.GetPNCJunctionById(object_map_id) != nullptr) {
        pnc_junctions_.emplace_back(overlap_ptr);
      }
    }
  }
}

void LaneInfo::CreateKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 16;

  segment_box_list_.clear();
  for (size_t id = 0; id < segments_.size(); ++id) {
    const auto &segment = segments_[id];
    segment_box_list_.emplace_back(AABox2d(segment.start(), segment.end()),
                                   this, &segment, id);
  }
  lane_segment_kdtree_.reset(new LaneSegmentKDTree(segment_box_list_, params));
}

MapAreaInfo::MapAreaInfo(const MapArea &map_area) : map_area_(map_area) {
  Init();
}

void MapAreaInfo::Init() {
  for (const auto &area_id : map_area_.cleanarea_id()) {
    clean_area_ids_.push_back(area_id.id());
  }
  return;
}

void MapAreaInfo::PostProcess(const HdMapImpl &map_instance) {
  for (const auto &id : clean_area_ids_) {
    const auto &area_id = MakeMapId(id);
    if (map_instance.GetCleanAreaById(area_id) != nullptr) {
      clean_areas_.push_back(map_instance.GetCleanAreaById(area_id));
    }
    if (map_instance.GetSlopeAreaById(area_id) != nullptr) {
      slope_areas_.push_back(map_instance.GetSlopeAreaById(area_id));
    }
    if (map_instance.GetElevatorAreaById(area_id) != nullptr) {
      elevator_areas_.push_back(map_instance.GetElevatorAreaById(area_id));
    }
    if (map_instance.GetNarrowAreaById(area_id) != nullptr) {
      narrow_areas_.push_back(map_instance.GetNarrowAreaById(area_id));
    }
    if (map_instance.GetMarkAreaById(area_id) != nullptr) {
      mark_areas_.push_back(map_instance.GetMarkAreaById(area_id));
    }
    if (map_instance.GetProhibitedAreaById(area_id) != nullptr) {
      prohibited_areas_.push_back(map_instance.GetProhibitedAreaById(area_id));
    }
    if (map_instance.GetPitAreaById(area_id) != nullptr) {
      pit_areas_.push_back(map_instance.GetPitAreaById(area_id));
    }
  }
}

bool MapAreaInfo::inCleanArea(const Vec2d &point, Id &id) const {
  for (size_t index = 0; index < clean_areas_.size(); index++) {
    if (clean_areas_[index]->inCleanArea(point)) {
      id = clean_areas_[index]->id();
      return true;
    }
  }
  return false;
}

bool MapAreaInfo::inPitArea(const Vec2d &point, Id &id) const {
  for (size_t index = 0; index < pit_areas_.size(); index++) {
    if (pit_areas_[index]->inPitArea(point)) {
      id = pit_areas_[index]->id();
      return true;
    }
  }
  return false;
}

bool MapAreaInfo::getPitBox(const math_utils::Vec2d &point,
                            cvte::hdmap::Box2d &box) const {
  for (size_t index = 0; index < pit_areas_.size(); index++) {
    if (pit_areas_[index]->inPitArea(point)) {
      pit_areas_[index]->getPitBox(point, box);
      return true;
    }
  }
  return false;
}

bool MapAreaInfo::inSlopeArea(const Vec2d &point, Id &id) const {
  for (size_t index = 0; index < slope_areas_.size(); index++) {
    if (slope_areas_[index]->inSlopeArea(point)) {
      id = slope_areas_[index]->id();
      return true;
    }
  }
  return false;
}
bool MapAreaInfo::inElevatorArea(const Vec2d &point, Id &id) const {
  for (size_t index = 0; index < elevator_areas_.size(); index++) {
    if (elevator_areas_[index]->inElevatorArea(point)) {
      id = elevator_areas_[index]->id();
      return true;
    }
  }
  return false;
}
bool MapAreaInfo::inNarrowArea(const Vec2d &point, Id &id) const {
  for (size_t index = 0; index < narrow_areas_.size(); index++) {
    if (narrow_areas_[index]->inNarrowArea(point)) {
      id = narrow_areas_[index]->id();
      return true;
    }
  }
  return false;
}
bool MapAreaInfo::inMarkArea(const Vec2d &point, Id &id) const {
  for (size_t index = 0; index < mark_areas_.size(); index++) {
    if (mark_areas_[index]->inMarkArea(point)) {
      id = mark_areas_[index]->id();
      return true;
    }
  }
  return false;
}
bool MapAreaInfo::inProhibitedArea(const Vec2d &point, Id &id) const {
  for (size_t index = 0; index < prohibited_areas_.size(); index++) {
    if (prohibited_areas_[index]->inProhibitedArea(point)) {
      id = prohibited_areas_[index]->id();
      return true;
    }
  }
  return false;
}

bool MapAreaInfo::inSpeedBumpArea(const Vec2d &point, Id &id) const {
  for (size_t index = 0; index < speed_bump_areas_.size(); index++) {
    if (speed_bump_areas_[index]->inSpeedBump(point)) {
      id = speed_bump_areas_[index]->id();
      return true;
    }
  }
  return false;
}

bool MapAreaInfo::inBlackArea(const Vec2d &point, Id &id) const {
  for (size_t index = 0; index < black_areas_.size(); index++) {
    if (black_areas_[index]->inBlackArea(point)) {
      id = black_areas_[index]->id();
      return true;
    }
  }
  return false;
}

CleanAreaInfo::CleanAreaInfo(const CleanArea &clean_area)
    : clean_area_(clean_area) {
  init();
}

void CleanAreaInfo::init() {
  polygon_ = ConvertToPolygon2d(clean_area_.polygon());
  if (clean_area_.has_speed()) {
    speed_ = clean_area_.speed();
  }
}

void CleanAreaInfo::postProcess(const HdMapImpl &map_instance) {
  return;
}

bool PitAreaInfo::getPitBox(const math_utils::Vec2d &point, Box2d &box) const {
  box = box_;
  return true;
}

PitAreaInfo::PitAreaInfo(const PitArea &pit_area) : pit_area_(pit_area) {
  init();
}

void PitAreaInfo::init() {
  polygon_ = ConvertToPolygon2d(pit_area_.polygon());
  box_ = polygon_.MinAreaBoundingBox();
}

void PitAreaInfo::postProcess(const HdMapImpl &map_instance) {
  return;
}

SlopeAreaInfo::SlopeAreaInfo(const SlopeArea &slope_area)
    : slope_area_(slope_area) {
  init();
}

void SlopeAreaInfo::init() {
  polygon_ = ConvertToPolygon2d(slope_area_.polygon());
  if (slope_area_.has_speed()) {
    speed_ = slope_area_.speed();
  }
  if (slope_area_.has_angle()) {
    angle_ = slope_area_.angle();
  }
  if (slope_area_.has_length()) {
    length_ = slope_area_.length();
  }
}

void SlopeAreaInfo::postProcess(const HdMapImpl &map_instance) {
  return;
}

ElevatorInfo::ElevatorInfo(const Elevator &elevator_area)
    : elevator_(elevator_area) {
  init();
}

void ElevatorInfo::init() {
  polygon_ = ConvertToPolygon2d(elevator_.polygon());
}

void ElevatorInfo::postProcess(const HdMapImpl &map_instance) {
  return;
}

NarrowAreaInfo::NarrowAreaInfo(const NarrowArea &narrow_area)
    : narrow_area_(narrow_area) {
  init();
}

void NarrowAreaInfo::init() {
  polygon_ = ConvertToPolygon2d(narrow_area_.polygon());
}

void NarrowAreaInfo::postProcess(const HdMapImpl &map_instance) {
  return;
}

MarkAreaInfo::MarkAreaInfo(const MarkArea &mark_area) : mark_area_(mark_area) {
  init();
}

void MarkAreaInfo::init() {
  polygon_ = ConvertToPolygon2d(mark_area_.polygon());
  if (mark_area_.has_speed()) {
    speed_ = mark_area_.speed();
  }
}

void MarkAreaInfo::postProcess(const HdMapImpl &map_instance) {
  return;
}

ProhibitedAreaInfo::ProhibitedAreaInfo(const ProhibitedArea &prohibited_area)
    : prohibited_area_(prohibited_area) {
  init();
}

void ProhibitedAreaInfo::init() {
  polygon_ = ConvertToPolygon2d(prohibited_area_.polygon());
}

void ProhibitedAreaInfo::postProcess(const HdMapImpl &map_instance) {
  return;
}

JunctionInfo::JunctionInfo(const Junction &junction) : junction_(junction) {
  Init();
}

void JunctionInfo::Init() {
  polygon_ = ConvertToPolygon2d(junction_.polygon());
  CHECK_GT(polygon_.num_points(), 2);

  for (const auto &overlap_id : junction_.overlap_id()) {
    overlap_ids_.emplace_back(overlap_id);
  }
}

void JunctionInfo::PostProcess(const HdMapImpl &map_instance) {
  UpdateOverlaps(map_instance);
}

void JunctionInfo::UpdateOverlaps(const HdMapImpl &map_instance) {
  for (const auto &overlap_id : overlap_ids_) {
    const auto &overlap_ptr = map_instance.GetOverlapById(overlap_id);
    if (overlap_ptr == nullptr) {
      continue;
    }

    for (const auto &object : overlap_ptr->overlap().object()) {
      const auto &object_id = object.id().id();
      if (object_id == id().id()) {
        continue;
      }

      if (object.has_stop_sign_overlap_info()) {
        overlap_stop_sign_ids_.push_back(object.id());
      }
    }
  }
}

SignalInfo::SignalInfo(const Signal &signal) : signal_(signal) {
  Init();
}

void SignalInfo::Init() {
  for (const auto &stop_line : signal_.stop_line()) {
    SegmentsFromCurve(stop_line, &segments_);
  }
  // ACHECK(!segments_.empty());
  std::vector<Vec2d> points;
  for (const auto &segment : segments_) {
    points.emplace_back(segment.start());
    points.emplace_back(segment.end());
  }
  CHECK_GT(points.size(), 0U);
}

CrosswalkInfo::CrosswalkInfo(const Crosswalk &crosswalk)
    : crosswalk_(crosswalk) {
  Init();
}

void CrosswalkInfo::Init() {
  polygon_ = ConvertToPolygon2d(crosswalk_.polygon());
  CHECK_GT(polygon_.num_points(), 2);
}

StopSignInfo::StopSignInfo(const StopSign &stop_sign) : stop_sign_(stop_sign) {
  init();
}

void StopSignInfo::init() {
  for (const auto &stop_line : stop_sign_.stop_line()) {
    SegmentsFromCurve(stop_line, &segments_);
  }
  // ACHECK(!segments_.empty());

  for (const auto &overlap_id : stop_sign_.overlap_id()) {
    overlap_ids_.emplace_back(overlap_id);
  }
}

void StopSignInfo::PostProcess(const HdMapImpl &map_instance) {
  UpdateOverlaps(map_instance);
}

void StopSignInfo::UpdateOverlaps(const HdMapImpl &map_instance) {
  for (const auto &overlap_id : overlap_ids_) {
    const auto &overlap_ptr = map_instance.GetOverlapById(overlap_id);
    if (overlap_ptr == nullptr) {
      continue;
    }

    for (const auto &object : overlap_ptr->overlap().object()) {
      const auto &object_id = object.id().id();
      if (object_id == id().id()) {
        continue;
      }

      if (object.has_junction_overlap_info()) {
        overlap_junction_ids_.push_back(object.id());
      } else if (object.has_lane_overlap_info()) {
        overlap_lane_ids_.push_back(object.id());
      }
    }
  }
  if (overlap_junction_ids_.empty()) {
    LOG(INFO) << "stop sign " << id().id()
              << "has no overlap with any junction.";
  }
}

YieldSignInfo::YieldSignInfo(const YieldSign &yield_sign)
    : yield_sign_(yield_sign) {
  Init();
}

void YieldSignInfo::Init() {
  for (const auto &stop_line : yield_sign_.stop_line()) {
    SegmentsFromCurve(stop_line, &segments_);
  }
  // segments_from_curve(yield_sign_.stop_line(), &segments_);
  // ACHECK(!segments_.empty());
}

ClearAreaInfo::ClearAreaInfo(const ClearArea &clear_area)
    : clear_area_(clear_area) {
  Init();
}

void ClearAreaInfo::Init() {
  polygon_ = ConvertToPolygon2d(clear_area_.polygon());
  CHECK_GT(polygon_.num_points(), 2);
}

SpeedBumpInfo::SpeedBumpInfo(const SpeedBump &speed_bump)
    : speed_bump_(speed_bump) {
  Init();
}

void SpeedBumpInfo::Init() {
  for (const auto &stop_line : speed_bump_.position()) {
    SegmentsFromCurve(stop_line, &segments_);
  }
  polygon_ = ConvertToPolygon2d(speed_bump_.polygon());
  // ACHECK(!segments_.empty());
}

BlackAreaInfo::BlackAreaInfo(const BlackArea &black_area)
    : black_area_(black_area) {
  Init();
}

void BlackAreaInfo::Init() {
  polygon_ = ConvertToPolygon2d(black_area_.polygon());
}

OverlapInfo::OverlapInfo(const Overlap &overlap) : overlap_(overlap) {}

const ObjectOverlapInfo *OverlapInfo::GetObjectOverlapInfo(const Id &id) const {
  for (const auto &object : overlap_.object()) {
    if (object.id().id() == id.id()) {
      return &object;
    }
  }
  return nullptr;
}

RoadInfo::RoadInfo(const Road &road) : road_(road) {
  for (const auto &section : road_.section()) {
    sections_.push_back(section);
    road_boundaries_.push_back(section.boundary());
  }
}

const std::vector<RoadBoundary> &RoadInfo::GetBoundaries() const {
  return road_boundaries_;
}

ParkingSpaceInfo::ParkingSpaceInfo(const ParkingSpace &parking_space)
    : parking_space_(parking_space) {
  Init();
}

void ParkingSpaceInfo::Init() {
  polygon_ = ConvertToPolygon2d(parking_space_.polygon());
  CHECK_GT(polygon_.num_points(), 2);
}

PNCJunctionInfo::PNCJunctionInfo(const PNCJunction &pnc_junction)
    : junction_(pnc_junction) {
  Init();
}

void PNCJunctionInfo::Init() {
  polygon_ = ConvertToPolygon2d(junction_.polygon());
  CHECK_GT(polygon_.num_points(), 2);

  for (const auto &overlap_id : junction_.overlap_id()) {
    overlap_ids_.emplace_back(overlap_id);
  }
}

}  // namespace hdmap
}  // namespace cvte
