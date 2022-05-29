#include "hdmap.hpp"

namespace cvte {
namespace hdmap {

std::shared_ptr<CvteHdMap> CvteHdMap::ptr_cvte_hdmap_ = nullptr;

int CvteHdMap::LoadMapFromFile(const std::string &map_filename) {
  map_id_.set_id(map_filename);
  return hdmap_impl_.LoadMapFromFile(map_filename);
}

std::shared_ptr<CvteHdMap> CvteHdMap::getInstance() {
  if (ptr_cvte_hdmap_ == nullptr) {
    ptr_cvte_hdmap_.reset(new CvteHdMap());
  }
  return ptr_cvte_hdmap_;
}

Id CvteHdMap::GetCurrentMapId() const {
  return map_id_;
}

LaneInfoConstPtr CvteHdMap::GetLaneById(const Id &id) const {
  return hdmap_impl_.GetLaneById(id);
}
JunctionInfoConstPtr CvteHdMap::GetJunctionById(const Id &id) const {
  return hdmap_impl_.GetJunctionById(id);
}
SignalInfoConstPtr CvteHdMap::GetSignalById(const Id &id) const {
  return hdmap_impl_.GetSignalById(id);
}
CrosswalkInfoConstPtr CvteHdMap::GetCrosswalkById(const Id &id) const {
  return hdmap_impl_.GetCrosswalkById(id);
}
StopSignInfoConstPtr CvteHdMap::GetStopSignById(const Id &id) const {
  return hdmap_impl_.GetStopSignById(id);
}
YieldSignInfoConstPtr CvteHdMap::GetYieldSignById(const Id &id) const {
  return hdmap_impl_.GetYieldSignById(id);
}
ClearAreaInfoConstPtr CvteHdMap::GetClearAreaById(const Id &id) const {
  return hdmap_impl_.GetClearAreaById(id);
}
SpeedBumpInfoConstPtr CvteHdMap::GetSpeedBumpById(const Id &id) const {
  return hdmap_impl_.GetSpeedBumpById(id);
}
BlackAreaInfoConstPtr CvteHdMap::GetBlackAreaById(const Id &id) const {
  return hdmap_impl_.GetBlackAreaById(id);
}
OverlapInfoConstPtr CvteHdMap::GetOverlapById(const Id &id) const {
  return hdmap_impl_.GetOverlapById(id);
}
RoadInfoConstPtr CvteHdMap::GetRoadById(const Id &id) const {
  return hdmap_impl_.GetRoadById(id);
}
ParkingSpaceInfoConstPtr CvteHdMap::GetParkingSpaceById(const Id &id) const {
  return hdmap_impl_.GetParkingSpaceById(id);
}
PNCJunctionInfoConstPtr CvteHdMap::GetPNCJunctionById(const Id &id) const {
  return hdmap_impl_.GetPNCJunctionById(id);
}
MapAreaInfoConstPtr CvteHdMap::GetMapAreaById(const Id &id) const {
  return hdmap_impl_.GetMapAreaById(id);
}
CleanAreaInfoConstPtr CvteHdMap::GetCleanAreaById(const Id &id) const {
  return hdmap_impl_.GetCleanAreaById(id);
}
SlopeAreaInfoConstPtr CvteHdMap::GetSlopeAreaById(const Id &id) const {
  return hdmap_impl_.GetSlopeAreaById(id);
}
ElevatorInfoConstPtr CvteHdMap::GetElevatorAreaById(const Id &id) const {
  return hdmap_impl_.GetElevatorAreaById(id);
}
NarrowAreaInfoConstPtr CvteHdMap::GetNarrowAreaById(const Id &id) const {
  return hdmap_impl_.GetNarrowAreaById(id);
}
ProhibitedAreaInfoConstPtr CvteHdMap::GetProhibitedAreaById(
    const Id &id) const {
  return hdmap_impl_.GetProhibitedAreaById(id);
}
MarkAreaInfoConstPtr CvteHdMap::GetMarkAreaById(const Id &id) const {
  return hdmap_impl_.GetMarkAreaById(id);
}

int CvteHdMap::GetLanes(const Vec2d &point, double distance,
                        std::vector<LaneInfoConstPtr> *lanes) const {
  return hdmap_impl_.GetLanes(point, distance, lanes);
}

int CvteHdMap::GetJunctions(
    const Vec2d &point, double distance,
    std::vector<JunctionInfoConstPtr> *junctions) const {
  return hdmap_impl_.GetJunctions(point, distance, junctions);
}

int CvteHdMap::GetCrosswalks(
    const Vec2d &point, double distance,
    std::vector<CrosswalkInfoConstPtr> *crosswalks) const {
  return hdmap_impl_.GetCrosswalks(point, distance, crosswalks);
}

int CvteHdMap::GetSignals(const Vec2d &point, double distance,
                          std::vector<SignalInfoConstPtr> *signals) const {
  return hdmap_impl_.GetSignals(point, distance, signals);
}

int CvteHdMap::GetStopSigns(
    const Vec2d &point, double distance,
    std::vector<StopSignInfoConstPtr> *stop_signs) const {
  return hdmap_impl_.GetStopSigns(point, distance, stop_signs);
}

int CvteHdMap::GetYieldSigns(
    const Vec2d &point, double distance,
    std::vector<YieldSignInfoConstPtr> *yield_signs) const {
  return hdmap_impl_.GetYieldSigns(point, distance, yield_signs);
}

int CvteHdMap::GetClearAreas(
    const Vec2d &point, double distance,
    std::vector<ClearAreaInfoConstPtr> *clear_areas) const {
  return hdmap_impl_.GetClearAreas(point, distance, clear_areas);
}

int CvteHdMap::GetSpeedBumps(
    const Vec2d &point, double distance,
    std::vector<SpeedBumpInfoConstPtr> *speed_bumps) const {
  return hdmap_impl_.GetSpeedBumps(point, distance, speed_bumps);
}

int CvteHdMap::GetBlackAreas(
    const Vec2d &point, double distance,
    std::vector<BlackAreaInfoConstPtr> *speed_bumps) const {
  return hdmap_impl_.GetBlackAreas(point, distance, speed_bumps);
}

int CvteHdMap::GetRoads(const Vec2d &point, double distance,
                        std::vector<RoadInfoConstPtr> *roads) const {
  return hdmap_impl_.GetRoads(point, distance, roads);
}

int CvteHdMap::GetParkingSpaces(
    const Vec2d &point, double distance,
    std::vector<ParkingSpaceInfoConstPtr> *parking_spaces) const {
  return hdmap_impl_.GetParkingSpaces(point, distance, parking_spaces);
}

int CvteHdMap::GetPNCJunctions(
    const Vec2d &point, double distance,
    std::vector<PNCJunctionInfoConstPtr> *pnc_junctions) const {
  return hdmap_impl_.GetPNCJunctions(point, distance, pnc_junctions);
}

int CvteHdMap::GetNearestLane(const Vec2d &point,
                              LaneInfoConstPtr *nearest_lane, double *nearest_s,
                              double *nearest_l) const {
  return hdmap_impl_.GetNearestLane(point, nearest_lane, nearest_s, nearest_l);
}

int CvteHdMap::GetNearestLaneWithHeading(
    const Vec2d &point, const double distance, const double central_heading,
    const double max_heading_difference, LaneInfoConstPtr *nearest_lane,
    double *nearest_s, double *nearest_l) const {
  return hdmap_impl_.GetNearestLaneWithHeading(
      point, distance, central_heading, max_heading_difference, nearest_lane,
      nearest_s, nearest_l);
}

int CvteHdMap::GetLanesWithHeading(const Vec2d &point, const double distance,
                                   const double central_heading,
                                   const double max_heading_difference,
                                   std::vector<LaneInfoConstPtr> *lanes) const {
  return hdmap_impl_.GetLanesWithHeading(point, distance, central_heading,
                                         max_heading_difference, lanes);
}

int CvteHdMap::GetRoadBoundaries(
    const Vec2d &point, double radius,
    std::vector<RoadROIBoundaryPtr> *road_boundaries,
    std::vector<JunctionBoundaryPtr> *junctions) const {
  return hdmap_impl_.GetRoadBoundaries(point, radius, road_boundaries,
                                       junctions);
}

int CvteHdMap::GetRoadBoundaries(
    const Vec2d &point, double radius, std::vector<RoadRoiPtr> *road_boundaries,
    std::vector<JunctionInfoConstPtr> *junctions) const {
  return hdmap_impl_.GetRoadBoundaries(point, radius, road_boundaries,
                                       junctions);
}

int CvteHdMap::GetRoi(const Vec2d &point, double radius,
                      std::vector<RoadRoiPtr> *roads_roi,
                      std::vector<PolygonRoiPtr> *polygons_roi) const {
  return hdmap_impl_.GetRoi(point, radius, roads_roi, polygons_roi);
}

int CvteHdMap::GetSlopeArea(
    const Vec2d &point, double radius,
    std::vector<SlopeAreaInfoConstPtr> *slope_areas) const {
  return hdmap_impl_.GetSlopeArea(point, radius, slope_areas);
}

int CvteHdMap::GetCleanArea(
    const Vec2d &point, double radius,
    std::vector<CleanAreaInfoConstPtr> *clean_areas) const {
  return hdmap_impl_.GetCleanArea(point, radius, clean_areas);
}

int CvteHdMap::GetNarrowArea(
    const Vec2d &point, double radius,
    std::vector<NarrowAreaInfoConstPtr> *narrow_areas) const {
  return hdmap_impl_.GetNarrowArea(point, radius, narrow_areas);
}

int CvteHdMap::GetElevatorArea(
    const Vec2d &point, double radius,
    std::vector<ElevatorInfoConstPtr> *elevator_areas) const {
  return hdmap_impl_.GetElevatorArea(point, radius, elevator_areas);
}

int CvteHdMap::GetMarkArea(
    const Vec2d &point, double radius,
    std::vector<MarkAreaInfoConstPtr> *mark_areas) const {
  return hdmap_impl_.GetMarkArea(point, radius, mark_areas);
}

int CvteHdMap::GetProhibitedArea(
    const Vec2d &point, double radius,
    std::vector<ProhibitedAreaInfoConstPtr> *prohibited_areas) const {
  return hdmap_impl_.GetProhibitedArea(point, radius, prohibited_areas);
}

int CvteHdMap::GetForwardNearestSignalsOnLane(
    const Vec2d &point, const double distance,
    std::vector<SignalInfoConstPtr> *signals) const {
  return hdmap_impl_.GetForwardNearestSignalsOnLane(point, distance, signals);
}

int CvteHdMap::GetStopSignAssociatedStopSigns(
    const Id &id, std::vector<StopSignInfoConstPtr> *stop_signs) const {
  return hdmap_impl_.GetStopSignAssociatedStopSigns(id, stop_signs);
}

int CvteHdMap::GetStopSignAssociatedLanes(
    const Id &id, std::vector<LaneInfoConstPtr> *lanes) const {
  return hdmap_impl_.GetStopSignAssociatedLanes(id, lanes);
}

int CvteHdMap::GetLocalMap(const Vec2d &point,
                           const std::pair<double, double> &range,
                           Map *local_map) const {
  return hdmap_impl_.GetLocalMap(point, range, local_map);
}

}  // namespace hdmap
}  // namespace cvte