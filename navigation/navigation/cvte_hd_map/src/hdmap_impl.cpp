#include "hdmap_impl.hpp"
#include <algorithm>
#include <limits>
#include <mutex>
#include <set>
#include <unordered_set>
#include "osm_adapter/osm_adapter.hpp"
#include "glog/logging.h"
namespace cvte {
namespace hdmap {
namespace {
using math_utils::Vec2d;

Id CreateHDMapId(const std::string &string_id) {
  Id id;
  id.set_id(string_id);
  return id;
}

// default lanes search radius in GetForwardNearestSignalsOnLane
constexpr double kLanesSearchRange = 10.0;
// backward search distance in GetForwardNearestSignalsOnLane
constexpr int kBackwardDistance = 4;

}  // namespace
int HdMapImpl::LoadMapFromFile(const std::string &map_filename) {
  Clear();
  Map pb_map;
  if (!OsmAdapter::LoadData(map_filename, &pb_map)) {
    LOG(ERROR) << "load data from file failed";
    return -1;
  }
  if (LoadMapFromProto(pb_map)) {
    LOG(ERROR) << "load map from proto failed";
    return -1;
  }
  return 0;
}

int HdMapImpl::LoadMapFromProto(const Map &map_proto) {
  if (&map_proto != &map_) {  // avoid an unnecessary copy
    Clear();
    map_ = map_proto;
  }
  for (const auto &lane : map_.lane()) {
    lane_table_[lane.id().id()].reset(new LaneInfo(lane));
  }
  for (const auto &junction : map_.junction()) {
    junction_table_[junction.id().id()].reset(new JunctionInfo(junction));
  }
  for (const auto &signal : map_.signal()) {
    signal_table_[signal.id().id()].reset(new SignalInfo(signal));
  }
  for (const auto &crosswalk : map_.crosswalk()) {
    crosswalk_table_[crosswalk.id().id()].reset(new CrosswalkInfo(crosswalk));
  }
  for (const auto &stop_sign : map_.stop_sign()) {
    stop_sign_table_[stop_sign.id().id()].reset(new StopSignInfo(stop_sign));
  }
  for (const auto &yield_sign : map_.yield()) {
    yield_sign_table_[yield_sign.id().id()].reset(
        new YieldSignInfo(yield_sign));
  }
  for (const auto &clear_area : map_.clear_area()) {
    clear_area_table_[clear_area.id().id()].reset(
        new ClearAreaInfo(clear_area));
  }
  for (const auto &speed_bump : map_.speed_bump()) {
    speed_bump_table_[speed_bump.id().id()].reset(
        new SpeedBumpInfo(speed_bump));
  }
  for (const auto &black_area : map_.black_area()) {
    black_area_table_[black_area.id().id()].reset(
        new BlackAreaInfo(black_area));
  }
  for (const auto &parking_space : map_.parking_space()) {
    parking_space_table_[parking_space.id().id()].reset(
        new ParkingSpaceInfo(parking_space));
  }
  for (const auto &pnc_junction : map_.pnc_junction()) {
    pnc_junction_table_[pnc_junction.id().id()].reset(
        new PNCJunctionInfo(pnc_junction));
  }
  for (const auto &overlap : map_.overlap()) {
    overlap_table_[overlap.id().id()].reset(new OverlapInfo(overlap));
  }

  for (const auto &road : map_.road()) {
    road_table_[road.id().id()].reset(new RoadInfo(road));
  }
  for (const auto &road_ptr_pair : road_table_) {
    const auto &road_id = road_ptr_pair.second->id();
    for (const auto &road_section : road_ptr_pair.second->sections()) {
      const auto &section_id = road_section.id();
      for (const auto &lane_id : road_section.lane_id()) {
        auto iter = lane_table_.find(lane_id.id());
        if (iter != lane_table_.end()) {
          iter->second->set_road_id(road_id);
          iter->second->set_section_id(section_id);
        } else {
          LOG(ERROR) << "Unknown lane id: " << lane_id.id();
        }
      }
    }
  }
  for (const auto &lane_ptr_pair : lane_table_) {
    lane_ptr_pair.second->PostProcess(*this);
  }
  for (const auto &junction_ptr_pair : junction_table_) {
    junction_ptr_pair.second->PostProcess(*this);
  }
  for (const auto &stop_sign_ptr_pair : stop_sign_table_) {
    stop_sign_ptr_pair.second->PostProcess(*this);
  }

  for (const auto &clean_area : map_.clean_area()) {
    clean_area_table_[clean_area.id().id()].reset(
        new CleanAreaInfo(clean_area));
  }
  for (const auto &pit_area : map_.pit_area()) {
    pit_area_table_[pit_area.id().id()].reset(new PitAreaInfo(pit_area));
  }
  for (const auto &slope_area : map_.slope_area()) {
    slope_area_table_[slope_area.id().id()].reset(
        new SlopeAreaInfo(slope_area));
  }
  for (const auto &elevator : map_.elevator()) {
    elevator_table_[elevator.id().id()].reset(new ElevatorInfo(elevator));
  }
  for (const auto &narrow_area : map_.narrow_area()) {
    narrow_area_table_[narrow_area.id().id()].reset(
        new NarrowAreaInfo(narrow_area));
  }
  for (const auto &mark_area : map_.mark_area()) {
    mark_area_table_[mark_area.id().id()].reset(new MarkAreaInfo(mark_area));
  }
  for (const auto &prohibited_area : map_.prohibited_area()) {
    prohibited_area_table_[prohibited_area.id().id()].reset(
        new ProhibitedAreaInfo(prohibited_area));
  }
  for (const auto &map_area : map_.map_area()) {
    map_area_table_[map_area.id().id()].reset(new MapAreaInfo(map_area));
  }
  for (const auto &map_area_pair : map_area_table_) {
    map_area_pair.second->PostProcess(*this);
  }

  BuildLaneSegmentKDTree();
  BuildJunctionPolygonKDTree();
  BuildSignalSegmentKDTree();
  BuildCrosswalkPolygonKDTree();
  BuildStopSignSegmentKDTree();
  BuildYieldSignSegmentKDTree();
  BuildClearAreaPolygonKDTree();
  BuildSpeedBumpSegmentKDTree();
  BuildParkingSpacePolygonKDTree();
  BuildPNCJunctionPolygonKDTree();
  BuildCleanAreaPolygonKDTree();
  // BuildMapAreaPolygonKDTree();
  BuildElevatorPolygonKDTree();
  BuildMarkPolygonKDTree();
  BuildSlopeAreaPolygonKDTree();
  BuildPitAreaPolygonKDTree();
  BuildNarrowAreaPolygonKDTree();
  BuildProhibitedPolygonKDTree();
  return 0;
}

LaneInfoConstPtr HdMapImpl::GetLaneById(const Id &id) const {
  LaneTable::const_iterator it = lane_table_.find(id.id());
  return it != lane_table_.end() ? it->second : nullptr;
}

JunctionInfoConstPtr HdMapImpl::GetJunctionById(const Id &id) const {
  JunctionTable::const_iterator it = junction_table_.find(id.id());
  return it != junction_table_.end() ? it->second : nullptr;
}

SignalInfoConstPtr HdMapImpl::GetSignalById(const Id &id) const {
  SignalTable::const_iterator it = signal_table_.find(id.id());
  return it != signal_table_.end() ? it->second : nullptr;
}

CrosswalkInfoConstPtr HdMapImpl::GetCrosswalkById(const Id &id) const {
  CrosswalkTable::const_iterator it = crosswalk_table_.find(id.id());
  return it != crosswalk_table_.end() ? it->second : nullptr;
}

StopSignInfoConstPtr HdMapImpl::GetStopSignById(const Id &id) const {
  StopSignTable::const_iterator it = stop_sign_table_.find(id.id());
  return it != stop_sign_table_.end() ? it->second : nullptr;
}

YieldSignInfoConstPtr HdMapImpl::GetYieldSignById(const Id &id) const {
  YieldSignTable::const_iterator it = yield_sign_table_.find(id.id());
  return it != yield_sign_table_.end() ? it->second : nullptr;
}

ClearAreaInfoConstPtr HdMapImpl::GetClearAreaById(const Id &id) const {
  ClearAreaTable::const_iterator it = clear_area_table_.find(id.id());
  return it != clear_area_table_.end() ? it->second : nullptr;
}

SpeedBumpInfoConstPtr HdMapImpl::GetSpeedBumpById(const Id &id) const {
  SpeedBumpTable::const_iterator it = speed_bump_table_.find(id.id());
  return it != speed_bump_table_.end() ? it->second : nullptr;
}

BlackAreaInfoConstPtr HdMapImpl::GetBlackAreaById(const Id &id) const {
  BlackAreaTable::const_iterator it = black_area_table_.find(id.id());
  return it != black_area_table_.end() ? it->second : nullptr;
}

OverlapInfoConstPtr HdMapImpl::GetOverlapById(const Id &id) const {
  OverlapTable::const_iterator it = overlap_table_.find(id.id());
  return it != overlap_table_.end() ? it->second : nullptr;
}

RoadInfoConstPtr HdMapImpl::GetRoadById(const Id &id) const {
  RoadTable::const_iterator it = road_table_.find(id.id());
  return it != road_table_.end() ? it->second : nullptr;
}

ParkingSpaceInfoConstPtr HdMapImpl::GetParkingSpaceById(const Id &id) const {
  ParkingSpaceTable::const_iterator it = parking_space_table_.find(id.id());
  return it != parking_space_table_.end() ? it->second : nullptr;
}

PNCJunctionInfoConstPtr HdMapImpl::GetPNCJunctionById(const Id &id) const {
  PNCJunctionTable::const_iterator it = pnc_junction_table_.find(id.id());
  return it != pnc_junction_table_.end() ? it->second : nullptr;
}

MapAreaInfoConstPtr HdMapImpl::GetMapAreaById(const Id &id) const {
  MapAreaTable::const_iterator it = map_area_table_.find(id.id());
  return it != map_area_table_.end() ? it->second : nullptr;
}

CleanAreaInfoConstPtr HdMapImpl::GetCleanAreaById(const Id &id) const {
  CleanAreaTable::const_iterator it = clean_area_table_.find(id.id());
  return it != clean_area_table_.end() ? it->second : nullptr;
}

PitAreaInfoConstPtr HdMapImpl::GetPitAreaById(const Id &id) const {
  PitAreaTable::const_iterator it = pit_area_table_.find(id.id());
  return it != pit_area_table_.end() ? it->second : nullptr;
}

SlopeAreaInfoConstPtr HdMapImpl::GetSlopeAreaById(const Id &id) const {
  SlopeAreaTable::const_iterator it = slope_area_table_.find(id.id());
  return it != slope_area_table_.end() ? it->second : nullptr;
}

ElevatorInfoConstPtr HdMapImpl::GetElevatorAreaById(const Id &id) const {
  ElevatorTable::const_iterator it = elevator_table_.find(id.id());
  return it != elevator_table_.end() ? it->second : nullptr;
}

NarrowAreaInfoConstPtr HdMapImpl::GetNarrowAreaById(const Id &id) const {
  NarrowAreaTable::const_iterator it = narrow_area_table_.find(id.id());
  return it != narrow_area_table_.end() ? it->second : nullptr;
}

ProhibitedAreaInfoConstPtr HdMapImpl::GetProhibitedAreaById(
    const Id &id) const {
  ProhibitedAreaTable::const_iterator it = prohibited_area_table_.find(id.id());
  return it != prohibited_area_table_.end() ? it->second : nullptr;
}

MarkAreaInfoConstPtr HdMapImpl::GetMarkAreaById(const Id &id) const {
  MarkAreaTable::const_iterator it = mark_area_table_.find(id.id());
  return it != mark_area_table_.end() ? it->second : nullptr;
}

int HdMapImpl::GetLanes(const Vec2d &point, double distance,
                        std::vector<LaneInfoConstPtr> *lanes) const {
  if (lanes == nullptr || lane_segment_kdtree_ == nullptr) {
    return -1;
  }
  lanes->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *lane_segment_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto &id : ids) {
    lanes->emplace_back(GetLaneById(CreateHDMapId(id)));
  }
  return 0;
}

int HdMapImpl::GetRoads(const Vec2d &point, double distance,
                        std::vector<RoadInfoConstPtr> *roads) const {
  std::vector<LaneInfoConstPtr> lanes;
  if (GetLanes(point, distance, &lanes) != 0) {
    return -1;
  }
  std::unordered_set<std::string> road_ids;
  for (auto &lane : lanes) {
    if (!lane->road_id().id().empty()) {
      road_ids.insert(lane->road_id().id());
    }
  }

  for (auto &road_id : road_ids) {
    RoadInfoConstPtr road = GetRoadById(CreateHDMapId(road_id));
    CHECK_NOTNULL(road);
    roads->push_back(road);
  }

  return 0;
}

int HdMapImpl::GetJunctions(
    const Vec2d &point, double distance,
    std::vector<JunctionInfoConstPtr> *junctions) const {
  if (junctions == nullptr || junction_polygon_kdtree_ == nullptr) {
    return -1;
  }
  junctions->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *junction_polygon_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto &id : ids) {
    junctions->emplace_back(GetJunctionById(CreateHDMapId(id)));
  }
  return 0;
}

int HdMapImpl::GetSignals(const Vec2d &point, double distance,
                          std::vector<SignalInfoConstPtr> *signals) const {
  if (signals == nullptr || signal_segment_kdtree_ == nullptr) {
    return -1;
  }
  signals->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *signal_segment_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto &id : ids) {
    signals->emplace_back(GetSignalById(CreateHDMapId(id)));
  }
  return 0;
}

int HdMapImpl::GetCrosswalks(
    const Vec2d &point, double distance,
    std::vector<CrosswalkInfoConstPtr> *crosswalks) const {
  if (crosswalks == nullptr || crosswalk_polygon_kdtree_ == nullptr) {
    return -1;
  }
  crosswalks->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *crosswalk_polygon_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto &id : ids) {
    crosswalks->emplace_back(GetCrosswalkById(CreateHDMapId(id)));
  }
  return 0;
}

int HdMapImpl::GetStopSigns(
    const Vec2d &point, double distance,
    std::vector<StopSignInfoConstPtr> *stop_signs) const {
  if (stop_signs == nullptr || stop_sign_segment_kdtree_ == nullptr) {
    return -1;
  }
  stop_signs->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *stop_sign_segment_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto &id : ids) {
    stop_signs->emplace_back(GetStopSignById(CreateHDMapId(id)));
  }
  return 0;
}

int HdMapImpl::GetYieldSigns(
    const Vec2d &point, double distance,
    std::vector<YieldSignInfoConstPtr> *yield_signs) const {
  if (yield_signs == nullptr || yield_sign_segment_kdtree_ == nullptr) {
    return -1;
  }
  yield_signs->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *yield_sign_segment_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto &id : ids) {
    yield_signs->emplace_back(GetYieldSignById(CreateHDMapId(id)));
  }

  return 0;
}

int HdMapImpl::GetClearAreas(
    const Vec2d &point, double distance,
    std::vector<ClearAreaInfoConstPtr> *clear_areas) const {
  if (clear_areas == nullptr || clear_area_polygon_kdtree_ == nullptr) {
    return -1;
  }
  clear_areas->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *clear_area_polygon_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto &id : ids) {
    clear_areas->emplace_back(GetClearAreaById(CreateHDMapId(id)));
  }

  return 0;
}

int HdMapImpl::GetSpeedBumps(
    const Vec2d &point, double distance,
    std::vector<SpeedBumpInfoConstPtr> *speed_bumps) const {
  if (speed_bumps == nullptr || speed_bump_segment_kdtree_ == nullptr) {
    return -1;
  }
  speed_bumps->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *speed_bump_segment_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto &id : ids) {
    speed_bumps->emplace_back(GetSpeedBumpById(CreateHDMapId(id)));
  }

  return 0;
}

int HdMapImpl::GetBlackAreas(
    const Vec2d &point, double distance,
    std::vector<BlackAreaInfoConstPtr> *black_areas) const {
  if (black_areas == nullptr || black_area_polygon_kdtree_ == nullptr) {
    return -1;
  }
  black_areas->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *black_area_polygon_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto &id : ids) {
    black_areas->emplace_back(GetBlackAreaById(CreateHDMapId(id)));
  }
  return 0;
}

int HdMapImpl::GetParkingSpaces(
    const Vec2d &point, double distance,
    std::vector<ParkingSpaceInfoConstPtr> *parking_spaces) const {
  if (parking_spaces == nullptr || parking_space_polygon_kdtree_ == nullptr) {
    return -1;
  }
  parking_spaces->clear();
  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *parking_space_polygon_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto &id : ids) {
    parking_spaces->emplace_back(GetParkingSpaceById(CreateHDMapId(id)));
  }

  return 0;
}

int HdMapImpl::GetPNCJunctions(
    const Vec2d &point, double distance,
    std::vector<PNCJunctionInfoConstPtr> *pnc_junctions) const {
  if (pnc_junctions == nullptr || pnc_junction_polygon_kdtree_ == nullptr) {
    return -1;
  }
  pnc_junctions->clear();

  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, distance, *pnc_junction_polygon_kdtree_, &ids);
  if (status < 0) {
    return status;
  }

  for (const auto &id : ids) {
    pnc_junctions->emplace_back(GetPNCJunctionById(CreateHDMapId(id)));
  }

  return 0;
}

int HdMapImpl::GetPitArea(const Vec2d &point, double radius,
                          std::vector<PitAreaInfoConstPtr> *pit_areas) const {
  if (pit_areas == nullptr || pit_polygon_kdtree_ == nullptr) {
    return -1;
  }
  pit_areas->clear();

  std::vector<std::string> ids;
  const int status = SearchObjects(point, radius, *pit_polygon_kdtree_, &ids);
  if (status < 0) {
    return status;
  }

  for (const auto &id : ids) {
    pit_areas->emplace_back(GetPitAreaById(CreateHDMapId(id)));
  }
  return 0;
}

int HdMapImpl::GetSlopeArea(
    const Vec2d &point, double radius,
    std::vector<SlopeAreaInfoConstPtr> *slope_areas) const {
  if (slope_areas == nullptr || slope_polygon_kdtree_ == nullptr) {
    return -1;
  }
  slope_areas->clear();

  std::vector<std::string> ids;
  const int status = SearchObjects(point, radius, *slope_polygon_kdtree_, &ids);
  if (status < 0) {
    return status;
  }

  for (const auto &id : ids) {
    slope_areas->emplace_back(GetSlopeAreaById(CreateHDMapId(id)));
  }
  return 0;
}

int HdMapImpl::GetCleanArea(
    const Vec2d &point, double radius,
    std::vector<CleanAreaInfoConstPtr> *clean_areas) const {
  if (clean_areas == nullptr || clean_polygon_kdtree_ == nullptr) {
    return -1;
  }
  clean_areas->clear();

  std::vector<std::string> ids;
  const int status = SearchObjects(point, radius, *clean_polygon_kdtree_, &ids);
  if (status < 0) {
    return status;
  }

  for (const auto &id : ids) {
    clean_areas->emplace_back(GetCleanAreaById(CreateHDMapId(id)));
  }
  return 0;
}

int HdMapImpl::GetNarrowArea(
    const Vec2d &point, double radius,
    std::vector<NarrowAreaInfoConstPtr> *narrow_areas) const {
  if (narrow_areas == nullptr || narrow_polygon_kdtree_ == nullptr) {
    return -1;
  }
  narrow_areas->clear();

  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, radius, *narrow_polygon_kdtree_, &ids);
  if (status < 0) {
    return status;
  }

  for (const auto &id : ids) {
    narrow_areas->push_back(GetNarrowAreaById(CreateHDMapId(id)));
  }
  return 0;
}

int HdMapImpl::GetElevatorArea(
    const Vec2d &point, double radius,
    std::vector<ElevatorInfoConstPtr> *elevator_areas) const {
  if (elevator_areas == nullptr || elevator_polygon_kdtree_ == nullptr) {
    return -1;
  }
  elevator_areas->clear();

  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, radius, *elevator_polygon_kdtree_, &ids);
  if (status < 0) {
    return status;
  }

  for (const auto &id : ids) {
    elevator_areas->push_back(GetElevatorAreaById(CreateHDMapId(id)));
  }
  return 0;
}

int HdMapImpl::GetMarkArea(
    const Vec2d &point, double radius,
    std::vector<MarkAreaInfoConstPtr> *mark_areas) const {
  if (mark_areas == nullptr || mark_polygon_kdtree_ == nullptr) {
    return -1;
  }
  mark_areas->clear();

  std::vector<std::string> ids;
  const int status = SearchObjects(point, radius, *mark_polygon_kdtree_, &ids);
  if (status < 0) {
    return status;
  }

  for (const auto &id : ids) {
    mark_areas->push_back(GetMarkAreaById(CreateHDMapId(id)));
  }
  return 0;
}

int HdMapImpl::GetProhibitedArea(
    const Vec2d &point, double radius,
    std::vector<ProhibitedAreaInfoConstPtr> *prohibited_areas) const {
  if (prohibited_areas == nullptr || prohibited_polygon_kdtree_ == nullptr) {
    return -1;
  }
  prohibited_areas->clear();

  std::vector<std::string> ids;
  const int status =
      SearchObjects(point, radius, *prohibited_polygon_kdtree_, &ids);
  if (status < 0) {
    return status;
  }
  for (const auto &id : ids) {
    prohibited_areas->push_back(GetProhibitedAreaById(CreateHDMapId(id)));
  }
  return 0;
}

int HdMapImpl::GetNearestLane(const Vec2d &point,
                              LaneInfoConstPtr *nearest_lane, double *nearest_s,
                              double *nearest_l) const {
  CHECK_NOTNULL(nearest_lane);
  CHECK_NOTNULL(nearest_s);
  CHECK_NOTNULL(nearest_l);
  const auto *segment_object = lane_segment_kdtree_->GetNearestObject(point);
  if (segment_object == nullptr) {
    return -1;
  }
  const Id &lane_id = segment_object->object()->id();
  *nearest_lane = GetLaneById(lane_id);
  //   ACHECK(*nearest_lane);
  const int id = segment_object->id();
  const auto &segment = (*nearest_lane)->segments()[id];
  Vec2d nearest_pt;
  segment.DistanceTo(point, &nearest_pt);
  *nearest_s = (*nearest_lane)->accumulate_s()[id] +
               nearest_pt.transpose() * (segment.start());
  // *nearest_l = segment.unit_direction().cross((point -
  // segment.start())).norm();
  *nearest_l = math_utils::CrossProd(
      segment.unit_direction().x(), segment.unit_direction().y(),
      (point - segment.start()).x(), (point - segment.start()).y());

  return 0;
}

int HdMapImpl::GetNearestLaneWithHeading(
    const Vec2d &point, const double distance, const double central_heading,
    const double max_heading_difference, LaneInfoConstPtr *nearest_lane,
    double *nearest_s, double *nearest_l) const {
  CHECK_NOTNULL(nearest_lane);
  CHECK_NOTNULL(nearest_s);
  CHECK_NOTNULL(nearest_l);

  std::vector<LaneInfoConstPtr> lanes;
  if (GetLanesWithHeading(point, distance, central_heading,
                          max_heading_difference, &lanes) != 0) {
    return -1;
  }

  double s = 0;
  size_t s_index = 0;
  Vec2d map_point;
  double min_distance = distance;
  for (const auto &lane : lanes) {
    double s_offset = 0.0;
    int s_offset_index = 0;
    double distance =
        lane->DistanceTo(point, &map_point, &s_offset, &s_offset_index);
    if (distance < min_distance) {
      min_distance = distance;
      *nearest_lane = lane;
      s = s_offset;
      s_index = s_offset_index;
    }
  }

  if (*nearest_lane == nullptr) {
    return -1;
  }

  *nearest_s = s;
  int segment_index = static_cast<int>(
      std::min(s_index, (*nearest_lane)->segments().size() - 1));
  const auto &segment_2d = (*nearest_lane)->segments()[segment_index];
  // *nearest_l =
  //     segment_2d.unit_direction().cross((point - segment_2d.start())).norm();

  *nearest_l = math_utils::CrossProd(
      segment_2d.unit_direction().x(), segment_2d.unit_direction().y(),
      (point - segment_2d.start()).x(), (point - segment_2d.start()).y());

  return 0;
}

int HdMapImpl::GetLanesWithHeading(const Vec2d &point, const double distance,
                                   const double central_heading,
                                   const double max_heading_difference,
                                   std::vector<LaneInfoConstPtr> *lanes) const {
  CHECK_NOTNULL(lanes);
  std::vector<LaneInfoConstPtr> all_lanes;
  const int status = GetLanes(point, distance, &all_lanes);
  if (status < 0 || all_lanes.empty()) {
    return -1;
  }

  lanes->clear();
  for (auto &lane : all_lanes) {
    Vec2d proj_pt(0.0, 0.0);
    double s_offset = 0.0;
    int s_offset_index = 0;
    double dis = lane->DistanceTo(point, &proj_pt, &s_offset, &s_offset_index);
    if (dis <= distance) {
      double heading_diff =
          fabs(lane->headings()[s_offset_index] - central_heading);
      if (fabs(math_utils::NormalizeAngle(heading_diff)) <=
          max_heading_difference) {
        lanes->push_back(lane);
      }
    }
  }

  return 0;
}

int HdMapImpl::GetRoadBoundaries(
    const Vec2d &point, double radius,
    std::vector<RoadROIBoundaryPtr> *road_boundaries,
    std::vector<JunctionBoundaryPtr> *junctions) const {
  CHECK_NOTNULL(road_boundaries);
  CHECK_NOTNULL(junctions);

  road_boundaries->clear();
  junctions->clear();

  std::vector<LaneInfoConstPtr> lanes;
  if (GetLanes(point, radius, &lanes) != 0 || lanes.empty()) {
    return -1;
  }

  std::unordered_set<std::string> junction_id_set;
  std::unordered_set<std::string> road_section_id_set;
  for (const auto &lane : lanes) {
    const auto road_id = lane->road_id();
    const auto section_id = lane->section_id();
    std::string unique_id = road_id.id() + section_id.id();
    if (road_section_id_set.count(unique_id) > 0) {
      continue;
    }
    road_section_id_set.insert(unique_id);
    const auto road_ptr = GetRoadById(road_id);
    if (road_ptr == nullptr) {
      LOG(ERROR) << "road id [" << road_id.id() << "] is not found.";
      continue;
    }
    if (road_ptr->has_junction_id()) {
      const Id junction_id = road_ptr->junction_id();
      if (junction_id_set.count(junction_id.id()) > 0) {
        continue;
      }
      junction_id_set.insert(junction_id.id());
      JunctionBoundaryPtr junction_boundary_ptr(new JunctionBoundary());
      junction_boundary_ptr->junction_info = GetJunctionById(junction_id);
      if (junction_boundary_ptr->junction_info == nullptr) {
        LOG(ERROR) << "junction id [" << junction_id.id() << "] is not found.";
        continue;
      }
      junctions->push_back(junction_boundary_ptr);
    } else {
      RoadROIBoundaryPtr road_boundary_ptr(new RoadROIBoundary());
      road_boundary_ptr->mutable_id()->CopyFrom(road_ptr->id());
      for (const auto &section : road_ptr->sections()) {
        if (section.id().id() == section_id.id()) {
          road_boundary_ptr->add_road_boundaries()->CopyFrom(
              section.boundary());
        }
      }
      road_boundaries->push_back(road_boundary_ptr);
    }
  }

  return 0;
}

int HdMapImpl::GetRoadBoundaries(
    const Vec2d &point, double radius, std::vector<RoadRoiPtr> *road_boundaries,
    std::vector<JunctionInfoConstPtr> *junctions) const {
  if (road_boundaries == nullptr || junctions == nullptr) {
    LOG(ERROR) << "the pointer in parameter is null";
    return -1;
  }
  road_boundaries->clear();
  junctions->clear();
  std::set<std::string> junction_id_set;
  std::vector<RoadInfoConstPtr> roads;
  if (GetRoads(point, radius, &roads) != 0) {
    LOG(ERROR) << "can not get roads in the range.";
    return -1;
  }
  for (const auto &road_ptr : roads) {
    if (road_ptr->has_junction_id()) {
      JunctionInfoConstPtr junction_ptr =
          GetJunctionById(road_ptr->junction_id());
      if (junction_id_set.find(junction_ptr->id().id()) ==
          junction_id_set.end()) {
        junctions->push_back(junction_ptr);
        junction_id_set.insert(junction_ptr->id().id());
      }
    } else {
      RoadRoiPtr road_boundary_ptr(new RoadRoi());
      const std::vector<cvte::hdmap::RoadBoundary> &temp_road_boundaries =
          road_ptr->GetBoundaries();
      road_boundary_ptr->id = road_ptr->id();
      for (const auto &temp_road_boundary : temp_road_boundaries) {
        cvte::hdmap::BoundaryPolygon boundary_polygon =
            temp_road_boundary.outer_polygon();
        for (const auto &edge : boundary_polygon.edge()) {
          if (edge.type() == cvte::hdmap::BoundaryEdge::LEFT_BOUNDARY) {
            for (const auto &s : edge.curve().segment()) {
              for (const auto &p : s.line_segment().point()) {
                road_boundary_ptr->left_boundary.line_points.push_back(
                    Vec2d(p.x(), p.y()));
              }
            }
          }
          if (edge.type() == cvte::hdmap::BoundaryEdge::RIGHT_BOUNDARY) {
            for (const auto &s : edge.curve().segment()) {
              for (const auto &p : s.line_segment().point()) {
                road_boundary_ptr->right_boundary.line_points.push_back(
                    Vec2d(p.x(), p.y()));
              }
            }
          }
        }
        if (temp_road_boundary.hole_size() != 0) {
          for (const auto &hole : temp_road_boundary.hole()) {
            PolygonBoundary hole_boundary;
            for (const auto &edge : hole.edge()) {
              if (edge.type() == cvte::hdmap::BoundaryEdge::NORMAL) {
                for (const auto &s : edge.curve().segment()) {
                  for (const auto &p : s.line_segment().point()) {
                    hole_boundary.polygon_points.push_back(Vec2d(p.x(), p.y()));
                  }
                }
              }
            }
            road_boundary_ptr->holes_boundary.push_back(hole_boundary);
          }
        }
      }
      road_boundaries->push_back(road_boundary_ptr);
    }
  }
  return 0;
}

int HdMapImpl::GetRoi(const Vec2d &point, double radius,
                      std::vector<RoadRoiPtr> *roads_roi,
                      std::vector<PolygonRoiPtr> *polygons_roi) const {
  if (roads_roi == nullptr || polygons_roi == nullptr) {
    LOG(ERROR) << "the pointer in parameter is null";
    return -1;
  }
  roads_roi->clear();
  polygons_roi->clear();
  std::set<std::string> polygon_id_set;
  std::vector<RoadInfoConstPtr> roads;
  std::vector<LaneInfoConstPtr> lanes;
  if (GetRoads(point, radius, &roads) != 0) {
    LOG(ERROR) << "can not get roads in the range.";
    return -1;
  }
  if (GetLanes(point, radius, &lanes) != 0) {
    LOG(ERROR) << "can not get lanes in the range.";
    return -1;
  }
  for (const auto &road_ptr : roads) {
    // get junction polygon
    if (road_ptr->has_junction_id()) {
      JunctionInfoConstPtr junction_ptr =
          GetJunctionById(road_ptr->junction_id());
      if (polygon_id_set.find(junction_ptr->id().id()) ==
          polygon_id_set.end()) {
        PolygonRoiPtr polygon_roi_ptr(new PolygonRoi());
        polygon_roi_ptr->polygon = junction_ptr->polygon();
        polygon_roi_ptr->attribute.type = PolygonType::JUNCTION_POLYGON;
        polygon_roi_ptr->attribute.id = junction_ptr->id();
        polygons_roi->push_back(polygon_roi_ptr);
        polygon_id_set.insert(junction_ptr->id().id());
      }
    } else {
      // get road boundary
      RoadRoiPtr road_boundary_ptr(new RoadRoi());
      std::vector<cvte::hdmap::RoadBoundary> temp_roads_roi;
      temp_roads_roi = road_ptr->GetBoundaries();
      if (!temp_roads_roi.empty()) {
        road_boundary_ptr->id = road_ptr->id();
        for (const auto &temp_road_boundary : temp_roads_roi) {
          cvte::hdmap::BoundaryPolygon boundary_polygon =
              temp_road_boundary.outer_polygon();
          for (const auto &edge : boundary_polygon.edge()) {
            if (edge.type() == cvte::hdmap::BoundaryEdge::LEFT_BOUNDARY) {
              for (const auto &s : edge.curve().segment()) {
                for (const auto &p : s.line_segment().point()) {
                  road_boundary_ptr->left_boundary.line_points.push_back(
                      Vec2d(p.x(), p.y()));
                }
              }
            }
            if (edge.type() == cvte::hdmap::BoundaryEdge::RIGHT_BOUNDARY) {
              for (const auto &s : edge.curve().segment()) {
                for (const auto &p : s.line_segment().point()) {
                  road_boundary_ptr->right_boundary.line_points.push_back(
                      Vec2d(p.x(), p.y()));
                }
              }
            }
          }
          if (temp_road_boundary.hole_size() != 0) {
            for (const auto &hole : temp_road_boundary.hole()) {
              PolygonBoundary hole_boundary;
              for (const auto &edge : hole.edge()) {
                if (edge.type() == cvte::hdmap::BoundaryEdge::NORMAL) {
                  for (const auto &s : edge.curve().segment()) {
                    for (const auto &p : s.line_segment().point()) {
                      hole_boundary.polygon_points.push_back(
                          Vec2d(p.x(), p.y()));
                    }
                  }
                }
              }
              road_boundary_ptr->holes_boundary.push_back(hole_boundary);
            }
          }
        }
        roads_roi->push_back(road_boundary_ptr);
      }
    }
  }

  for (const auto &lane_ptr : lanes) {
    // get parking space polygon
    for (const auto &overlap_id : lane_ptr->lane().overlap_id()) {
      OverlapInfoConstPtr overlap_ptr = GetOverlapById(overlap_id);
      for (int i = 0; i < overlap_ptr->overlap().object_size(); ++i) {
        if (overlap_ptr->overlap().object(i).id().id() == lane_ptr->id().id()) {
          continue;
        } else {
          ParkingSpaceInfoConstPtr parkingspace_ptr =
              GetParkingSpaceById(overlap_ptr->overlap().object(i).id());
          if (parkingspace_ptr != nullptr) {
            if (polygon_id_set.find(parkingspace_ptr->id().id()) ==
                polygon_id_set.end()) {
              PolygonRoiPtr polygon_roi_ptr(new PolygonRoi());
              polygon_roi_ptr->polygon = parkingspace_ptr->polygon();
              polygon_roi_ptr->attribute.type =
                  PolygonType::PARKINGSPACE_POLYGON;
              polygon_roi_ptr->attribute.id = parkingspace_ptr->id();
              polygons_roi->push_back(polygon_roi_ptr);
              polygon_id_set.insert(parkingspace_ptr->id().id());
            }
          }
        }
      }
    }
  }
  return 0;
}

int HdMapImpl::GetForwardNearestSignalsOnLane(
    const Vec2d &point, const double distance,
    std::vector<SignalInfoConstPtr> *signals) const {
  CHECK_NOTNULL(signals);

  signals->clear();
  LaneInfoConstPtr lane_ptr = nullptr;
  double nearest_s = 0.0;
  double nearest_l = 0.0;

  std::vector<LaneInfoConstPtr> temp_surrounding_lanes;
  std::vector<LaneInfoConstPtr> surrounding_lanes;
  int s_index = 0;
  Vec2d car_point;
  car_point = point;
  Vec2d map_point;
  if (GetLanes(point, kLanesSearchRange, &temp_surrounding_lanes) == -1) {
    LOG(INFO) << "Can not find lanes around car.";
    return -1;
  }
  for (const auto &surround_lane : temp_surrounding_lanes) {
    if (surround_lane->IsOnLane(car_point)) {
      surrounding_lanes.push_back(surround_lane);
    }
  }
  if (surrounding_lanes.empty()) {
    LOG(INFO) << "Car is not on lane.";
    return -1;
  }
  for (const auto &lane : surrounding_lanes) {
    if (!lane->signals().empty()) {
      lane_ptr = lane;
      nearest_l =
          lane_ptr->DistanceTo(car_point, &map_point, &nearest_s, &s_index);
      break;
    }
  }
  if (lane_ptr == nullptr) {
    GetNearestLane(point, &lane_ptr, &nearest_s, &nearest_l);
    if (lane_ptr == nullptr) {
      return -1;
    }
  }

  double unused_distance = distance + kBackwardDistance;
  double back_distance = kBackwardDistance;
  double s = nearest_s;
  while (s < back_distance) {
    for (const auto &predecessor_lane_id : lane_ptr->lane().predecessor_id()) {
      lane_ptr = GetLaneById(predecessor_lane_id);
      if (lane_ptr->lane().turn() == cvte::hdmap::Lane::NO_TURN) {
        break;
      }
    }
    back_distance = back_distance - s;
    s = lane_ptr->total_length();
  }
  double s_start = s - back_distance;
  while (lane_ptr != nullptr) {
    double signal_min_dist = std::numeric_limits<double>::infinity();
    std::vector<SignalInfoConstPtr> min_dist_signal_ptr;
    for (const auto &overlap_id : lane_ptr->lane().overlap_id()) {
      OverlapInfoConstPtr overlap_ptr = GetOverlapById(overlap_id);
      double lane_overlap_offset_s = 0.0;
      SignalInfoConstPtr signal_ptr = nullptr;
      for (int i = 0; i < overlap_ptr->overlap().object_size(); ++i) {
        if (overlap_ptr->overlap().object(i).id().id() == lane_ptr->id().id()) {
          lane_overlap_offset_s =
              overlap_ptr->overlap().object(i).lane_overlap_info().start_s() -
              s_start;
          continue;
        }
        signal_ptr = GetSignalById(overlap_ptr->overlap().object(i).id());
        if (signal_ptr == nullptr || lane_overlap_offset_s < 0.0) {
          break;
        }
        if (lane_overlap_offset_s < signal_min_dist) {
          signal_min_dist = lane_overlap_offset_s;
          min_dist_signal_ptr.clear();
          min_dist_signal_ptr.push_back(signal_ptr);
        } else if (lane_overlap_offset_s < (signal_min_dist + 0.1) &&
                   lane_overlap_offset_s > (signal_min_dist - 0.1)) {
          min_dist_signal_ptr.push_back(signal_ptr);
        }
      }
    }
    if (!min_dist_signal_ptr.empty() && unused_distance >= signal_min_dist) {
      *signals = min_dist_signal_ptr;
      break;
    }
    unused_distance = unused_distance - (lane_ptr->total_length() - s_start);
    if (unused_distance <= 0) {
      break;
    }
    LaneInfoConstPtr tmp_lane_ptr = nullptr;
    for (const auto &successor_lane_id : lane_ptr->lane().successor_id()) {
      tmp_lane_ptr = GetLaneById(successor_lane_id);
      if (tmp_lane_ptr->lane().turn() == cvte::hdmap::Lane::NO_TURN) {
        break;
      }
    }
    lane_ptr = tmp_lane_ptr;
    s_start = 0;
  }
  return 0;
}

int HdMapImpl::GetStopSignAssociatedStopSigns(
    const Id &id, std::vector<StopSignInfoConstPtr> *stop_signs) const {
  CHECK_NOTNULL(stop_signs);

  const auto &stop_sign = GetStopSignById(id);
  if (stop_sign == nullptr) {
    return -1;
  }

  std::vector<Id> associate_stop_sign_ids;
  const auto junction_ids = stop_sign->OverlapJunctionIds();
  for (const auto &junction_id : junction_ids) {
    const auto &junction = GetJunctionById(junction_id);
    if (junction == nullptr) {
      continue;
    }
    const auto stop_sign_ids = junction->OverlapStopSignIds();
    std::copy(stop_sign_ids.begin(), stop_sign_ids.end(),
              std::back_inserter(associate_stop_sign_ids));
  }

  std::vector<Id> associate_lane_ids;
  for (const auto &stop_sign_id : associate_stop_sign_ids) {
    if (stop_sign_id.id() == id.id()) {
      // exclude current stop sign
      continue;
    }
    const auto &stop_sign = GetStopSignById(stop_sign_id);
    if (stop_sign == nullptr) {
      continue;
    }
    stop_signs->push_back(stop_sign);
  }

  return 0;
}

int HdMapImpl::GetStopSignAssociatedLanes(
    const Id &id, std::vector<LaneInfoConstPtr> *lanes) const {
  CHECK_NOTNULL(lanes);

  const auto &stop_sign = GetStopSignById(id);
  if (stop_sign == nullptr) {
    return -1;
  }

  std::vector<Id> associate_stop_sign_ids;
  const auto junction_ids = stop_sign->OverlapJunctionIds();
  for (const auto &junction_id : junction_ids) {
    const auto &junction = GetJunctionById(junction_id);
    if (junction == nullptr) {
      continue;
    }
    const auto stop_sign_ids = junction->OverlapStopSignIds();
    std::copy(stop_sign_ids.begin(), stop_sign_ids.end(),
              std::back_inserter(associate_stop_sign_ids));
  }

  std::vector<Id> associate_lane_ids;
  for (const auto &stop_sign_id : associate_stop_sign_ids) {
    if (stop_sign_id.id() == id.id()) {
      // exclude current stop sign
      continue;
    }
    const auto &stop_sign = GetStopSignById(stop_sign_id);
    if (stop_sign == nullptr) {
      continue;
    }
    const auto lane_ids = stop_sign->OverlapLaneIds();
    std::copy(lane_ids.begin(), lane_ids.end(),
              std::back_inserter(associate_lane_ids));
  }

  for (const auto lane_id : associate_lane_ids) {
    const auto &lane = GetLaneById(lane_id);
    if (lane == nullptr) {
      continue;
    }
    lanes->push_back(lane);
  }

  return 0;
}

int HdMapImpl::GetLocalMap(const Vec2d &point,
                           const std::pair<double, double> &range,
                           Map *local_map) const {
  CHECK_NOTNULL(local_map);

  double distance = std::max(range.first, range.second);
  //   CHECK_GT(distance, 0.0);

  std::vector<LaneInfoConstPtr> lanes;
  GetLanes(point, distance, &lanes);

  std::vector<JunctionInfoConstPtr> junctions;
  GetJunctions(point, distance, &junctions);

  std::vector<CrosswalkInfoConstPtr> crosswalks;
  GetCrosswalks(point, distance, &crosswalks);

  std::vector<SignalInfoConstPtr> signals;
  GetSignals(point, distance, &signals);

  std::vector<StopSignInfoConstPtr> stop_signs;
  GetStopSigns(point, distance, &stop_signs);

  std::vector<YieldSignInfoConstPtr> yield_signs;
  GetYieldSigns(point, distance, &yield_signs);

  std::vector<ClearAreaInfoConstPtr> clear_areas;
  GetClearAreas(point, distance, &clear_areas);

  std::vector<SpeedBumpInfoConstPtr> speed_bumps;
  GetSpeedBumps(point, distance, &speed_bumps);

  std::vector<BlackAreaInfoConstPtr> black_areas;
  GetBlackAreas(point, distance, &black_areas);

  std::vector<RoadInfoConstPtr> roads;
  GetRoads(point, distance, &roads);

  std::vector<ParkingSpaceInfoConstPtr> parking_spaces;
  GetParkingSpaces(point, distance, &parking_spaces);

  std::vector<SlopeAreaInfoConstPtr> slope_areas;
  GetSlopeArea(point, distance, &slope_areas);

  std::vector<PitAreaInfoConstPtr> pit_areas;
  GetPitArea(point, distance, &pit_areas);

  std::vector<ElevatorInfoConstPtr> elevator_areas;
  GetElevatorArea(point, distance, &elevator_areas);

  std::vector<ProhibitedAreaInfoConstPtr> prohibited_areas;
  GetProhibitedArea(point, distance, &prohibited_areas);

  std::vector<MarkAreaInfoConstPtr> mark_areas;
  GetMarkArea(point, distance, &mark_areas);

  std::vector<CleanAreaInfoConstPtr> clean_areas;
  GetCleanArea(point, distance, &clean_areas);

  std::vector<NarrowAreaInfoConstPtr> narrow_areas;
  GetNarrowArea(point, distance, &narrow_areas);

  std::unordered_set<std::string> map_element_ids;
  std::vector<Id> overlap_ids;

  for (auto &lane_ptr : lanes) {
    map_element_ids.insert(lane_ptr->id().id());
    std::copy(lane_ptr->lane().overlap_id().begin(),
              lane_ptr->lane().overlap_id().end(),
              std::back_inserter(overlap_ids));
    *local_map->add_lane() = lane_ptr->lane();
  }

  for (auto &crosswalk_ptr : crosswalks) {
    map_element_ids.insert(crosswalk_ptr->id().id());
    std::copy(crosswalk_ptr->crosswalk().overlap_id().begin(),
              crosswalk_ptr->crosswalk().overlap_id().end(),
              std::back_inserter(overlap_ids));
    *local_map->add_crosswalk() = crosswalk_ptr->crosswalk();
  }

  for (auto &junction_ptr : junctions) {
    map_element_ids.insert(junction_ptr->id().id());
    std::copy(junction_ptr->junction().overlap_id().begin(),
              junction_ptr->junction().overlap_id().end(),
              std::back_inserter(overlap_ids));
    *local_map->add_junction() = junction_ptr->junction();
  }

  for (auto &signal_ptr : signals) {
    map_element_ids.insert(signal_ptr->id().id());
    std::copy(signal_ptr->signal().overlap_id().begin(),
              signal_ptr->signal().overlap_id().end(),
              std::back_inserter(overlap_ids));
    *local_map->add_signal() = signal_ptr->signal();
  }

  for (auto &stop_sign_ptr : stop_signs) {
    map_element_ids.insert(stop_sign_ptr->id().id());
    std::copy(stop_sign_ptr->stop_sign().overlap_id().begin(),
              stop_sign_ptr->stop_sign().overlap_id().end(),
              std::back_inserter(overlap_ids));
    *local_map->add_stop_sign() = stop_sign_ptr->stop_sign();
  }

  for (auto &yield_sign_ptr : yield_signs) {
    std::copy(yield_sign_ptr->yield_sign().overlap_id().begin(),
              yield_sign_ptr->yield_sign().overlap_id().end(),
              std::back_inserter(overlap_ids));
    map_element_ids.insert(yield_sign_ptr->id().id());
    *local_map->add_yield() = yield_sign_ptr->yield_sign();
  }

  for (auto &clear_area_ptr : clear_areas) {
    map_element_ids.insert(clear_area_ptr->id().id());
    std::copy(clear_area_ptr->clear_area().overlap_id().begin(),
              clear_area_ptr->clear_area().overlap_id().end(),
              std::back_inserter(overlap_ids));
    *local_map->add_clear_area() = clear_area_ptr->clear_area();
  }

  for (auto &speed_bump_ptr : speed_bumps) {
    map_element_ids.insert(speed_bump_ptr->id().id());
    std::copy(speed_bump_ptr->speed_bump().overlap_id().begin(),
              speed_bump_ptr->speed_bump().overlap_id().end(),
              std::back_inserter(overlap_ids));
    *local_map->add_speed_bump() = speed_bump_ptr->speed_bump();
  }

  // for(auto &black_area_ptr : black_areas) {
  //   map_element_ids.insert(black_area_ptr->id().id());
  //   std::copy(black_area_ptr->black_area())
  // }

  for (auto &road_ptr : roads) {
    map_element_ids.insert(road_ptr->id().id());
    *local_map->add_road() = road_ptr->road();
  }

  for (auto &parking_space_ptr : parking_spaces) {
    map_element_ids.insert(parking_space_ptr->id().id());
    std::copy(parking_space_ptr->parking_space().overlap_id().begin(),
              parking_space_ptr->parking_space().overlap_id().end(),
              std::back_inserter(overlap_ids));
    *local_map->add_parking_space() = parking_space_ptr->parking_space();
  }

  for (auto &clean_area_ptr : clean_areas) {
    map_element_ids.insert(clean_area_ptr->id().id());
    *local_map->add_clean_area() = clean_area_ptr->cleanArea();
  }

  for (auto &slope_area_ptr : slope_areas) {
    map_element_ids.insert(slope_area_ptr->id().id());
    *local_map->add_slope_area() = slope_area_ptr->slopeArea();
  }

  for (auto &pit_area_ptr : pit_areas) {
    map_element_ids.insert(pit_area_ptr->id().id());
    *local_map->add_pit_area() = pit_area_ptr->pitArea();
  }

  for (auto &narrow_area_ptr : narrow_areas) {
    map_element_ids.insert(narrow_area_ptr->id().id());
    *local_map->add_narrow_area() = narrow_area_ptr->narrowArea();
  }

  for (auto &elevator_ptr : elevator_areas) {
    map_element_ids.insert(elevator_ptr->id().id());
    *local_map->add_elevator() = elevator_ptr->elevator();
  }

  for (auto &prohibited_area_ptr : prohibited_areas) {
    map_element_ids.insert(prohibited_area_ptr->id().id());
    *local_map->add_prohibited_area() = prohibited_area_ptr->prohibitedArea();
  }

  for (auto &mark_area_ptr : mark_areas) {
    map_element_ids.insert(mark_area_ptr->id().id());
    *local_map->add_mark_area() = mark_area_ptr->markArea();
  }

  for (auto &overlap_id : overlap_ids) {
    auto overlap_ptr = GetOverlapById(overlap_id);
    if (overlap_ptr == nullptr) {
      LOG(ERROR) << "overlpa id [" << overlap_id.id() << "] is not found.";
      continue;
    }

    bool need_delete = false;
    for (auto &overlap_object : overlap_ptr->overlap().object()) {
      if (map_element_ids.count(overlap_object.id().id()) <= 0) {
        need_delete = true;
      }
    }

    if (!need_delete) {
      *local_map->add_overlap() = overlap_ptr->overlap();
    }
  }

  return 0;
}

template <class Table, class BoxTable, class KDTree>
void HdMapImpl::BuildSegmentKDTree(const Table &table,
                                   const AABoxKDTreeParams &params,
                                   BoxTable *const box_table,
                                   std::unique_ptr<KDTree> *const kdtree) {
  box_table->clear();
  for (const auto &info_with_id : table) {
    const auto *info = info_with_id.second.get();
    for (size_t id = 0; id < info->segments().size(); ++id) {
      const auto &segment = info->segments()[id];
      box_table->emplace_back(AABox2d(segment.start(), segment.end()), info,
                              &segment, id);
    }
  }
  kdtree->reset(new KDTree(*box_table, params));
}

template <class Table, class BoxTable, class KDTree>
void HdMapImpl::BuildPolygonKDTree(const Table &table,
                                   const AABoxKDTreeParams &params,
                                   BoxTable *const box_table,
                                   std::unique_ptr<KDTree> *const kdtree) {
  box_table->clear();
  for (const auto &info_with_id : table) {
    const auto *info = info_with_id.second.get();
    const auto &polygon = info->polygon();
    box_table->emplace_back(polygon.AABoundingBox(), info, &polygon, 0);
  }
  kdtree->reset(new KDTree(*box_table, params));
}

void HdMapImpl::BuildLaneSegmentKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 16;
  BuildSegmentKDTree(lane_table_, params, &lane_segment_boxes_,
                     &lane_segment_kdtree_);
}

void HdMapImpl::BuildJunctionPolygonKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 1;
  BuildPolygonKDTree(junction_table_, params, &junction_polygon_boxes_,
                     &junction_polygon_kdtree_);
}

void HdMapImpl::BuildCrosswalkPolygonKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 1;
  BuildPolygonKDTree(crosswalk_table_, params, &crosswalk_polygon_boxes_,
                     &crosswalk_polygon_kdtree_);
}

void HdMapImpl::BuildSignalSegmentKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 4;
  BuildSegmentKDTree(signal_table_, params, &signal_segment_boxes_,
                     &signal_segment_kdtree_);
}

void HdMapImpl::BuildStopSignSegmentKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 4;
  BuildSegmentKDTree(stop_sign_table_, params, &stop_sign_segment_boxes_,
                     &stop_sign_segment_kdtree_);
}

void HdMapImpl::BuildYieldSignSegmentKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 4;
  BuildSegmentKDTree(yield_sign_table_, params, &yield_sign_segment_boxes_,
                     &yield_sign_segment_kdtree_);
}

void HdMapImpl::BuildClearAreaPolygonKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 4;
  BuildPolygonKDTree(clear_area_table_, params, &clear_area_polygon_boxes_,
                     &clear_area_polygon_kdtree_);
}

void HdMapImpl::BuildSpeedBumpSegmentKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 4;
  BuildSegmentKDTree(speed_bump_table_, params, &speed_bump_segment_boxes_,
                     &speed_bump_segment_kdtree_);
}

void HdMapImpl::BuildBlackAreaPolygonKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;
  params.max_leaf_size = 4;
  BuildPolygonKDTree(black_area_table_, params, &black_area_polygon_boxes_,
                     &black_area_polygon_kdtree_);
}

void HdMapImpl::BuildParkingSpacePolygonKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 4;
  BuildPolygonKDTree(parking_space_table_, params,
                     &parking_space_polygon_boxes_,
                     &parking_space_polygon_kdtree_);
}

void HdMapImpl::BuildPNCJunctionPolygonKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 1;
  BuildPolygonKDTree(pnc_junction_table_, params, &pnc_junction_polygon_boxes_,
                     &pnc_junction_polygon_kdtree_);
}

// void HdMapImpl::BuildMapAreaPolygonKDTree() {
//   AABoxKDTreeParams params;
//   params.max_leaf_dimension = 5.0;  // meters.
//   params.max_leaf_size = 1;
//   BuildPolygonKDTree(map_area_table_, params, &map_polygon_boxes_,
//                      &map_polygon_kdtree_);
// }

void HdMapImpl::BuildCleanAreaPolygonKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 1;
  BuildPolygonKDTree(clean_area_table_, params, &clean_polygon_boxes_,
                     &clean_polygon_kdtree_);
}
void HdMapImpl::BuildSlopeAreaPolygonKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 1;
  BuildPolygonKDTree(slope_area_table_, params, &slope_polygon_boxed_,
                     &slope_polygon_kdtree_);
}
void HdMapImpl::BuildPitAreaPolygonKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 1;
  BuildPolygonKDTree(pit_area_table_, params, &pit_polygon_boxed_,
                     &pit_polygon_kdtree_);
}
void HdMapImpl::BuildNarrowAreaPolygonKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 1;
  BuildPolygonKDTree(narrow_area_table_, params, &narrow_polygon_boxed_,
                     &narrow_polygon_kdtree_);
}
void HdMapImpl::BuildElevatorPolygonKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 1;
  BuildPolygonKDTree(elevator_table_, params, &elevator_polygon_boxed_,
                     &elevator_polygon_kdtree_);
}
void HdMapImpl::BuildMarkPolygonKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 1;
  BuildPolygonKDTree(mark_area_table_, params, &mark_polygon_boxed_,
                     &mark_polygon_kdtree_);
}
void HdMapImpl::BuildProhibitedPolygonKDTree() {
  AABoxKDTreeParams params;
  params.max_leaf_dimension = 5.0;  // meters.
  params.max_leaf_size = 1;
  BuildPolygonKDTree(prohibited_area_table_, params, &prohibited_polygon_boxed_,
                     &prohibited_polygon_kdtree_);
}

template <class KDTree>
int HdMapImpl::SearchObjects(const Vec2d &center, const double radius,
                             const KDTree &kdtree,
                             std::vector<std::string> *const results) {
  //   static std::mutex mutex_search_object;
  //   std::unique_lock<std::mutex> lock(mutex_search_object);
  if (results == nullptr) {
    return -1;
  }
  auto objects = kdtree.GetObjects(center, radius);
  std::unordered_set<std::string> result_ids;
  result_ids.reserve(objects.size());
  for (const auto *object_ptr : objects) {
    result_ids.insert(object_ptr->object()->id().id());
  }

  results->reserve(result_ids.size());
  results->assign(result_ids.begin(), result_ids.end());
  return 0;
}

void HdMapImpl::Clear() {
  //   map_.Clear();
  lane_table_.clear();
  junction_table_.clear();
  signal_table_.clear();
  crosswalk_table_.clear();
  stop_sign_table_.clear();
  yield_sign_table_.clear();
  overlap_table_.clear();

  clean_area_table_.clear();
  slope_area_table_.clear();
  elevator_table_.clear();
  narrow_area_table_.clear();
  mark_area_table_.clear();
  prohibited_area_table_.clear();
  map_area_table_.clear();

  lane_segment_boxes_.clear();
  lane_segment_kdtree_.reset(nullptr);
  junction_polygon_boxes_.clear();
  junction_polygon_kdtree_.reset(nullptr);
  crosswalk_polygon_boxes_.clear();
  crosswalk_polygon_kdtree_.reset(nullptr);
  signal_segment_boxes_.clear();
  signal_segment_kdtree_.reset(nullptr);
  stop_sign_segment_boxes_.clear();
  stop_sign_segment_kdtree_.reset(nullptr);
  yield_sign_segment_boxes_.clear();
  yield_sign_segment_kdtree_.reset(nullptr);
  clear_area_polygon_boxes_.clear();
  clear_area_polygon_kdtree_.reset(nullptr);
  speed_bump_segment_boxes_.clear();
  speed_bump_segment_kdtree_.reset(nullptr);
  parking_space_polygon_boxes_.clear();
  parking_space_polygon_kdtree_.reset(nullptr);
  pnc_junction_polygon_boxes_.clear();
  pnc_junction_polygon_kdtree_.reset(nullptr);

  clean_polygon_boxes_.clear();
  clean_polygon_kdtree_.reset(nullptr);
  slope_polygon_boxed_.clear();
  slope_polygon_kdtree_.reset(nullptr);
  pit_polygon_boxed_.clear();
  pit_polygon_kdtree_.reset(nullptr);
  elevator_polygon_boxed_.clear();
  elevator_polygon_kdtree_.reset(nullptr);
  narrow_polygon_boxed_.clear();
  narrow_polygon_kdtree_.reset(nullptr);
  mark_polygon_boxed_.clear();
  mark_polygon_kdtree_.reset(nullptr);
  prohibited_polygon_boxed_.clear();
  prohibited_polygon_kdtree_.reset(nullptr);
}
}  // namespace hdmap
}  // namespace cvte