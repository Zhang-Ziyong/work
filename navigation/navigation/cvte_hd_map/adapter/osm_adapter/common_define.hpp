/**
 * @file common_define.hpp
 * @author linyanlong(linyanlong@cvte.com)
 * @brief
 * @version 0.1
 * @date 2021-12-23
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef COMMON_DEFINE_HPP_
#define COMMON_DEFINE_HPP_
#include <string>
#include "map_clear_area.pb.h"
#include "map_crosswalk.pb.h"
#include "map_id.pb.h"
#include "map_junction.pb.h"
#include "map_lane.pb.h"
#include "map_overlap.pb.h"
#include "map_parking_space.pb.h"
#include "map_pnc_junction.pb.h"
#include "map_road.pb.h"
#include "map_signal.pb.h"
#include "map_speed_bump.pb.h"
#include "map_stop_sign.pb.h"
#include "map_yield_sign.pb.h"
#include "map_rsu.pb.h"
#include "map_c5_area.pb.h"
namespace cvte {
namespace hdmap {
using PbRoad = cvte::hdmap::Road;
using PbRoadSection = cvte::hdmap::RoadSection;
using PbLane = cvte::hdmap::Lane;
using PbJunction = cvte::hdmap::Junction;
using PbSignal = cvte::hdmap::Signal;
using PbSubSignal = cvte::hdmap::Subsignal;
using PbCrosswalk = cvte::hdmap::Crosswalk;
using PbParkingSpace = cvte::hdmap::ParkingSpace;
using PbSpeedBump = cvte::hdmap::SpeedBump;
using PbStopSign = cvte::hdmap::StopSign;
using PbYieldSign = cvte::hdmap::YieldSign;
using PbObjectOverlapInfo = cvte::hdmap::ObjectOverlapInfo;
using PbOverlap = cvte::hdmap::Overlap;
using PbClearArea = cvte::hdmap::ClearArea;
using PbLineSegment = cvte::hdmap::LineSegment;
using PbCurveSegment = cvte::hdmap::CurveSegment;
using PbCurve = cvte::hdmap::Curve;
using PbLaneType = cvte::hdmap::Lane_LaneType;
using PbTurnType = cvte::hdmap::Lane_LaneTurn;
using PbID = cvte::hdmap::Id;
using PbLaneBoundary = cvte::hdmap::LaneBoundary;
using PbLaneBoundaryTypeType = cvte::hdmap::LaneBoundaryType_Type;
using PbPolygon = cvte::hdmap::Polygon;
using PbBoundaryPolygon = cvte::hdmap::BoundaryPolygon;
using PbBoundaryEdge = cvte::hdmap::BoundaryEdge;
using PbRegionOverlap = cvte::hdmap::RegionOverlapInfo;
using PbPNCJunction = cvte::hdmap::PNCJunction;
using PbSlopeArea = cvte::hdmap::SlopeArea;
using PbPitArea = cvte::hdmap::PitArea;
using PbCleanArea = cvte::hdmap::CleanArea;
using PbElevator = cvte::hdmap::Elevator;
using PbNarrowArea = cvte::hdmap::NarrowArea;
using PbMarkArea = cvte::hdmap::MarkArea;
using PbProhibitedArea = cvte::hdmap::ProhibitedArea;
using PbBlackArea = cvte::hdmap::BlackArea;

using PbLaneDirection = cvte::hdmap::Lane_LaneDirection;
using PbSignalType = cvte::hdmap::Signal_Type;
using PbSubSignalType = cvte::hdmap::Subsignal_Type;
using PbStopSignType = cvte::hdmap::StopSign_StopType;
using PbBoundaryEdgeType = cvte::hdmap::BoundaryEdge_Type;
using PbRoadType = cvte::hdmap::Road_Type;
using PbSignInfoType = cvte::hdmap::SignInfo::Type;
using PbPassageType = cvte::hdmap::Passage_Type;
using PbPassageGroup = cvte::hdmap::PassageGroup;

struct StopLineInternal {
  std::string id;
  PbCurve curve;
};

struct StopSignInternal {
  std::string id;
  PbStopSign stop_sign;
  std::unordered_set<std::string> stop_line_ids;
};

struct YieldSignInternal {
  std::string id;
  PbYieldSign yield_sign;
  std::unordered_set<std::string> stop_line_ids;
};

struct TrafficLightInternal {
  std::string id;
  PbSignal traffic_light;
  std::unordered_set<std::string> stop_line_ids;
};

struct OverlapWithLane {
  std::string object_id;
  double start_s;
  double end_s;
  bool is_merge;

  std::string region_overlap_id;
  std::vector<PbRegionOverlap> region_overlaps;
};

struct OverlapWithJunction {
  std::string object_id;
};

struct LaneInternal {
  PbLane lane;
  std::vector<OverlapWithLane> overlap_signals;
  std::vector<OverlapWithLane> overlap_objects;
  std::vector<OverlapWithLane> overlap_junctions;
  std::vector<OverlapWithLane> overlap_lanes;
};

struct JunctionInternal {
  PbJunction junction;
  std::unordered_set<std::string> road_ids;
  std::vector<OverlapWithJunction> overlap_with_junctions;
};

struct RoadSectionInternal {
  std::string id;
  PbRoadSection section;
  std::vector<LaneInternal> lanes;
};

struct RoadInternal {
  std::string id;
  PbRoad road;

  bool in_junction;
  std::string junction_id;

  std::string type;

  std::vector<RoadSectionInternal> sections;

  std::vector<TrafficLightInternal> traffic_lights;
  // std::vector<RSUInternal> rsus;
  std::vector<StopSignInternal> stop_signs;
  std::vector<YieldSignInternal> yield_signs;
  std::vector<PbCrosswalk> crosswalks;
  std::vector<PbClearArea> clear_areas;
  std::vector<PbSpeedBump> speed_bumps;
  std::vector<StopLineInternal> stop_lines;
  std::vector<PbParkingSpace> parking_spaces;
  std::vector<PbPNCJunction> pnc_junctions;

  RoadInternal() : in_junction(false) { junction_id = ""; }
};
}  // namespace hdmap
}  // namespace cvte

#endif