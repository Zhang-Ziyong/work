/**
 * @file hd_common.hpp
 * @author linyanlong (linyanlong@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-12-13
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef _HD_COMMON_HPP_
#define _HD_COMMON_HPP_
#include <vector>
#include <memory>
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

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "polygon2d.h"
#include "aabox2d.h"
#include "box2d.h"
#include "aaboxkdtree2d.h"

namespace cvte {
namespace hdmap {
// using Vec2d = Eigen::Vector2d;
// using Id = int;

template <class Object, class GeoObject>
class ObjectWithAABox {
 public:
  ObjectWithAABox(const AABox2d &aabox, const Object *object,
                  const GeoObject *geo_object, const int id)
      : aabox_(aabox), object_(object), geo_object_(geo_object), id_(id) {}
  ~ObjectWithAABox() {}
  const AABox2d &aabox() const { return aabox_; }
  double DistanceTo(const Vec2d &point) const {
    return geo_object_->DistanceTo(point);
  }
  double DistanceSquareTo(const Vec2d &point) const {
    return geo_object_->DistanceSquareTo(point);
  }
  const Object *object() const { return object_; }
  const GeoObject *geo_object() const { return geo_object_; }
  int id() const { return id_; }

 private:
  AABox2d aabox_;
  const Object *object_;
  const GeoObject *geo_object_;
  int id_;
};

class LaneInfo;
// class LaneSegmentBox;
// class LaneSegmentKDTree;
class JunctionInfo;
class CrosswalkInfo;
class SignalInfo;
class StopSignInfo;
class YieldSignInfo;
class OverlapInfo;
class ClearAreaInfo;
class SpeedBumpInfo;
class RoadInfo;
class ParkingSpaceInfo;
class PNCJunctionInfo;
class MapAreaInfo;
class CleanAreaInfo;
class PitAreaInfo;
class SlopeAreaInfo;
class ElevatorInfo;
class NarrowAreaInfo;
class MarkAreaInfo;
class ProhibitedAreaInfo;
class BlackAreaInfo;

class HdMapImpl;

struct LineBoundary {
  std::vector<Vec2d> line_points;
};
struct PolygonBoundary {
  std::vector<Vec2d> polygon_points;
};

enum class PolygonType {
  JUNCTION_POLYGON = 0,
  PARKINGSPACE_POLYGON = 1,
  ROAD_HOLE_POLYGON = 2,
};

struct RoiAttribute {
  PolygonType type;
  Id id;
};

struct PolygonRoi {
  Polygon2d polygon;
  RoiAttribute attribute;
};

struct RoadRoi {
  Id id;
  LineBoundary left_boundary;
  LineBoundary right_boundary;
  std::vector<PolygonBoundary> holes_boundary;
};

using LaneSegmentBox = ObjectWithAABox<LaneInfo, LineSegment2d>;
using LaneSegmentKDTree = AABoxKDTree2d<LaneSegmentBox>;
using OverlapInfoConstPtr = std::shared_ptr<const OverlapInfo>;
using LaneInfoConstPtr = std::shared_ptr<const LaneInfo>;
using JunctionInfoConstPtr = std::shared_ptr<const JunctionInfo>;
using SignalInfoConstPtr = std::shared_ptr<const SignalInfo>;
using CrosswalkInfoConstPtr = std::shared_ptr<const CrosswalkInfo>;
using StopSignInfoConstPtr = std::shared_ptr<const StopSignInfo>;
using YieldSignInfoConstPtr = std::shared_ptr<const YieldSignInfo>;
using ClearAreaInfoConstPtr = std::shared_ptr<const ClearAreaInfo>;
using SpeedBumpInfoConstPtr = std::shared_ptr<const SpeedBumpInfo>;
using BlackAreaInfoConstPtr = std::shared_ptr<const BlackAreaInfo>;
using RoadInfoConstPtr = std::shared_ptr<const RoadInfo>;
using ParkingSpaceInfoConstPtr = std::shared_ptr<const ParkingSpaceInfo>;
using PolygonRoiPtr = std::shared_ptr<PolygonRoi>;
using RoadRoiPtr = std::shared_ptr<RoadRoi>;
using RoadROIBoundaryPtr = std::shared_ptr<RoadROIBoundary>;
using PNCJunctionInfoConstPtr = std::shared_ptr<const PNCJunctionInfo>;
using MapAreaInfoConstPtr = std::shared_ptr<const MapAreaInfo>;
using CleanAreaInfoConstPtr = std::shared_ptr<const CleanAreaInfo>;
using SlopeAreaInfoConstPtr = std::shared_ptr<const SlopeAreaInfo>;
using ElevatorInfoConstPtr = std::shared_ptr<const ElevatorInfo>;
using NarrowAreaInfoConstPtr = std::shared_ptr<const NarrowAreaInfo>;
using MarkAreaInfoConstPtr = std::shared_ptr<const MarkAreaInfo>;
using ProhibitedAreaInfoConstPtr = std::shared_ptr<const ProhibitedAreaInfo>;
using PitAreaInfoConstPtr = std::shared_ptr<const PitAreaInfo>;

// class Lane {
//   public:
//     Id id() const { return id_; }
//     Id id() { return id_; }
//   private:
//     Id id_;
// };

class LaneInfo {
 public:
  explicit LaneInfo(const Lane &lane);

  const Id &id() const { return lane_.id(); }
  const Id &road_id() const { return road_id_; }
  const Id &section_id() const { return section_id_; }
  const Lane &lane() const { return lane_; }
  const std::vector<Vec2d> &points() const { return points_; }
  const std::vector<Vec2d> &unit_directions() const { return unit_directions_; }
  double Heading(const double s) const;
  double Curvature(const double s) const;
  const std::vector<double> &headings() const { return headings_; }
  const std::vector<LineSegment2d> &segments() const { return segments_; }
  const std::vector<double> &accumulate_s() const { return accumulated_s_; }
  const std::vector<OverlapInfoConstPtr> &overlaps() const { return overlaps_; }
  const std::vector<OverlapInfoConstPtr> &cross_lanes() const {
    return cross_lanes_;
  }
  const std::vector<OverlapInfoConstPtr> &signals() const { return signals_; }
  const std::vector<OverlapInfoConstPtr> &yield_signs() const {
    return yield_signs_;
  }
  const std::vector<OverlapInfoConstPtr> &stop_signs() const {
    return stop_signs_;
  }
  const std::vector<OverlapInfoConstPtr> &crosswalks() const {
    return crosswalks_;
  }
  const std::vector<OverlapInfoConstPtr> &junctions() const {
    return junctions_;
  }
  const std::vector<OverlapInfoConstPtr> &clear_areas() const {
    return clear_areas_;
  }
  const std::vector<OverlapInfoConstPtr> &speed_bumps() const {
    return speed_bumps_;
  }
  const std::vector<OverlapInfoConstPtr> &parking_spaces() const {
    return parking_spaces_;
  }
  const std::vector<OverlapInfoConstPtr> &pnc_junctions() const {
    return pnc_junctions_;
  }
  double total_length() const { return total_length_; }
  using SampledWidth = std::pair<double, double>;
  const std::vector<SampledWidth> &sampled_left_width() const {
    return sampled_left_width_;
  }
  const std::vector<SampledWidth> &sampled_right_width() const {
    return sampled_right_width_;
  }
  void GetWidth(const double s, double *left_width, double *right_width) const;
  double GetWidth(const double s) const;
  double GetEffectiveWidth(const double s) const;

  const std::vector<SampledWidth> &sampled_left_road_width() const {
    return sampled_left_road_width_;
  }
  const std::vector<SampledWidth> &sampled_right_road_width() const {
    return sampled_right_road_width_;
  }
  void GetRoadWidth(const double s, double *left_width,
                    double *right_width) const;
  double GetRoadWidth(const double s) const;

  bool IsOnLane(const Vec2d &point) const;
  bool IsOnLane(const Box2d &box) const;

  Vec2d GetSmoothPoint(double s) const;
  double DistanceTo(const Vec2d &point) const;
  double DistanceTo(const Vec2d &point, Vec2d *map_point, double *s_offset,
                    int *s_offset_index) const;
  Vec2d GetNearestPoint(const Vec2d &point, double *distance) const;
  bool GetProjection(const Vec2d &point, double *accumulate_s,
                     double *lateral) const;

 private:
  friend class HdMapImpl;
  friend class RoadInfo;
  void Init();
  void PostProcess(const HdMapImpl &map_instance);
  void UpdateOverlaps(const HdMapImpl &map_instance);
  double GetWidthFromSample(const std::vector<LaneInfo::SampledWidth> &samples,
                            const double s) const;
  void CreateKDTree();
  void set_road_id(const Id &road_id) { road_id_ = road_id; }
  void set_section_id(const Id &section_id) { section_id_ = section_id; }

 private:
  const Lane &lane_;
  std::vector<Vec2d> points_;
  std::vector<Vec2d> unit_directions_;
  std::vector<double> headings_;
  std::vector<LineSegment2d> segments_;
  std::vector<double> accumulated_s_;
  std::vector<std::string> overlap_ids_;
  std::vector<OverlapInfoConstPtr> overlaps_;
  std::vector<OverlapInfoConstPtr> cross_lanes_;
  std::vector<OverlapInfoConstPtr> signals_;
  std::vector<OverlapInfoConstPtr> yield_signs_;
  std::vector<OverlapInfoConstPtr> stop_signs_;
  std::vector<OverlapInfoConstPtr> crosswalks_;
  std::vector<OverlapInfoConstPtr> junctions_;
  std::vector<OverlapInfoConstPtr> clear_areas_;
  std::vector<OverlapInfoConstPtr> speed_bumps_;
  std::vector<OverlapInfoConstPtr> parking_spaces_;
  std::vector<OverlapInfoConstPtr> pnc_junctions_;
  double total_length_ = 0.0;
  std::vector<SampledWidth> sampled_left_width_;
  std::vector<SampledWidth> sampled_right_width_;

  std::vector<SampledWidth> sampled_left_road_width_;
  std::vector<SampledWidth> sampled_right_road_width_;

  std::vector<LaneSegmentBox> segment_box_list_;
  std::unique_ptr<LaneSegmentKDTree> lane_segment_kdtree_;

  Id road_id_;
  Id section_id_;
};

class MapAreaInfo {
 public:
  explicit MapAreaInfo(const MapArea &map_area);

  Id id() const { return map_area_.id(); }

  const std::vector<CleanAreaInfoConstPtr> &cleanAreas() const {
    return clean_areas_;
  }
  const std::vector<PitAreaInfoConstPtr> &pitAreas() const {
    return pit_areas_;
  }
  const std::vector<SlopeAreaInfoConstPtr> &slopeAreas() const {
    return slope_areas_;
  }
  const std::vector<ElevatorInfoConstPtr> &elecatorAreas() const {
    return elevator_areas_;
  }
  const std::vector<NarrowAreaInfoConstPtr> &narrowAreas() const {
    return narrow_areas_;
  }
  const std::vector<MarkAreaInfoConstPtr> &MarkAreas() const {
    return mark_areas_;
  }
  const std::vector<ProhibitedAreaInfoConstPtr> &prohibitedAreas() const {
    return prohibited_areas_;
  }
  const std::vector<SpeedBumpInfoConstPtr> &speedBumpAreas() const {
    return speed_bump_areas_;
  }
  const std::vector<BlackAreaInfoConstPtr> &blackAreas() const {
    return black_areas_;
  }

  bool inCleanArea(const Vec2d &point, Id &id) const;
  bool inSlopeArea(const Vec2d &point, Id &id) const;
  bool inElevatorArea(const Vec2d &point, Id &id) const;
  bool inNarrowArea(const Vec2d &point, Id &id) const;
  bool inMarkArea(const Vec2d &point, Id &id) const;
  bool inProhibitedArea(const Vec2d &point, Id &id) const;
  bool inSpeedBumpArea(const Vec2d &point, Id &id) const;
  bool inBlackArea(const Vec2d &point, Id &id) const;
  bool inPitArea(const Vec2d &point, Id &id) const;
  bool getPitBox(const math_utils::Vec2d &point, cvte::hdmap::Box2d &box) const;

 private:
  friend class HdMapImpl;
  std::vector<std::string> clean_area_ids_;
  void Init();
  void PostProcess(const HdMapImpl &map_instance);

 private:
  const MapArea &map_area_;
  std::vector<CleanAreaInfoConstPtr> clean_areas_;
  std::vector<SlopeAreaInfoConstPtr> slope_areas_;
  std::vector<ElevatorInfoConstPtr> elevator_areas_;
  std::vector<NarrowAreaInfoConstPtr> narrow_areas_;
  std::vector<MarkAreaInfoConstPtr> mark_areas_;
  std::vector<ProhibitedAreaInfoConstPtr> prohibited_areas_;
  std::vector<SpeedBumpInfoConstPtr> speed_bump_areas_;
  std::vector<BlackAreaInfoConstPtr> black_areas_;
  std::vector<PitAreaInfoConstPtr> pit_areas_;
};
using MapPolygonBox = ObjectWithAABox<MapAreaInfo, Polygon2d>;
using MapPolygonKDTree = AABoxKDTree2d<MapPolygonBox>;

class CleanAreaInfo {
 public:
  explicit CleanAreaInfo(const CleanArea &clean_area);

  const Id &id() const { return clean_area_.id(); }
  const Polygon2d &polygon() const { return polygon_; }
  const CleanArea &cleanArea() const { return clean_area_; }
  double speed() const { return speed_; }
  bool hasSpeed() const { return clean_area_.has_speed(); }
  std::string material() const { return clean_area_.material(); }
  bool hasMaterial() const { return clean_area_.has_material(); }
  std::string color() const { return clean_area_.color(); }
  bool hasColor() const { return clean_area_.has_color(); }
  std::string getStatic() const { return clean_area_.static_area(); }
  bool hasStaticArea() const { return clean_area_.has_static_area(); }
  bool inCleanArea(const Vec2d &point) const {
    return polygon_.IsPointIn(point);
  }

 private:
  friend class hdMapImpl;
  void init();
  void postProcess(const HdMapImpl &map_instance);

 private:
  const CleanArea &clean_area_;
  Polygon2d polygon_;
  double speed_;
};
using CleanPolygonBox = ObjectWithAABox<CleanAreaInfo, Polygon2d>;
using CleanPolygonKDTree = AABoxKDTree2d<CleanPolygonBox>;

class SlopeAreaInfo {
 public:
  explicit SlopeAreaInfo(const SlopeArea &clean_area);

  const Id &id() const { return slope_area_.id(); }
  const Polygon2d &polygon() const { return polygon_; }
  const SlopeArea &slopeArea() const { return slope_area_; }
  double speed() const { return speed_; }
  bool hasSpeed() const { return slope_area_.has_speed(); }
  double length() const { return length_; }
  bool hasLength() const { return slope_area_.has_length(); }
  double angle() const { return angle_; }
  bool hasAngle() const { return slope_area_.has_angle(); }
  std::string material() const { return slope_area_.material(); }
  bool hasMaterial() const { return slope_area_.has_material(); }
  std::string color() const { return slope_area_.color(); }
  bool hasColor() const { return slope_area_.has_color(); }
  std::string getStatic() const { return slope_area_.static_area(); }
  bool hasStaticArea() const { return slope_area_.has_static_area(); }
  bool inSlopeArea(const Vec2d &point) const {
    return polygon_.IsPointIn(point);
  }

 private:
  friend class hdMapImpl;
  void init();
  void postProcess(const HdMapImpl &map_instance);

 private:
  const SlopeArea &slope_area_;
  Polygon2d polygon_;
  double speed_;
  double length_;
  double angle_;
};
using PitPolygonBox = ObjectWithAABox<PitAreaInfo, Polygon2d>;
using PitPolygonKDTree = AABoxKDTree2d<PitPolygonBox>;
class PitAreaInfo {
 public:
  explicit PitAreaInfo(const PitArea &pit_area);
  const Id &id() const { return pit_area_.id(); }
  const Polygon2d &polygon() const { return polygon_; }
  const PitArea &pitArea() const { return pit_area_; }
  double speed() const { return pit_area_.speed(); }
  bool hasSpeed() const { return pit_area_.has_speed(); }
  double length() const { return pit_area_.length(); }
  bool hasLength() const { return pit_area_.has_length(); }
  double angle() const { return pit_area_.angle(); }
  bool hasAngle() const { return pit_area_.has_angle(); }
  std::string material() const { return pit_area_.material(); }
  bool hasMaterial() const { return pit_area_.has_material(); }
  std::string color() const { return pit_area_.color(); }
  bool hasColor() const { return pit_area_.has_color(); }
  std::string getStatic() const { return pit_area_.static_area(); }
  bool hasStaticArea() const { return pit_area_.has_static_area(); }
  bool inPitArea(const Vec2d &point) const { return polygon_.IsPointIn(point); }
  bool getPitBox(const math_utils::Vec2d &point, Box2d &box) const;

 private:
  friend class hdMapImpl;
  void init();
  void postProcess(const HdMapImpl &map_instance);

 private:
  const PitArea &pit_area_;
  Polygon2d polygon_;
  Box2d box_;
};

using SlopePolygonBox = ObjectWithAABox<SlopeAreaInfo, Polygon2d>;
using SlopePolygonKDTree = AABoxKDTree2d<SlopePolygonBox>;

class ElevatorInfo {
 public:
  explicit ElevatorInfo(const Elevator &elevator_area);

  const Id &id() const { return elevator_.id(); }
  const Polygon2d &polygon() const { return polygon_; }
  const Elevator &elevator() const { return elevator_; }
  bool inElevatorArea(const Vec2d &point) const {
    return polygon_.IsPointIn(point);
  }

 private:
  friend class hdMapImpl;
  void init();
  void postProcess(const HdMapImpl &map_instance);

 private:
  const Elevator &elevator_;
  Polygon2d polygon_;
};
using ElevatorPolygonBox = ObjectWithAABox<ElevatorInfo, Polygon2d>;
using ElevatorPolygonKDTree = AABoxKDTree2d<ElevatorPolygonBox>;

class NarrowAreaInfo {
 public:
  explicit NarrowAreaInfo(const NarrowArea &narrow_area);

  const Id &id() const { return narrow_area_.id(); }
  const Polygon2d &polygon() const { return polygon_; }
  const NarrowArea &narrowArea() const { return narrow_area_; }
  double speed() const { return narrow_area_.speed(); }
  bool hasSpeed() const { return narrow_area_.has_speed(); }
  std::string material() const { return narrow_area_.material(); }
  bool hasMaterial() const { return narrow_area_.has_material(); }
  std::string color() const { return narrow_area_.color(); }
  bool hasColor() const { return narrow_area_.has_color(); }
  std::string getStatic() const { return narrow_area_.static_area(); }
  bool hasStaticArea() const { return narrow_area_.has_static_area(); }
  bool inNarrowArea(const Vec2d &point) const {
    return polygon_.IsPointIn(point);
  }

 private:
  friend class hdMapImpl;
  void init();
  void postProcess(const HdMapImpl &map_instance);

 private:
  const NarrowArea &narrow_area_;
  Polygon2d polygon_;
};
using NarrowPolygonBox = ObjectWithAABox<NarrowAreaInfo, Polygon2d>;
using NarrowPolygonKDTree = AABoxKDTree2d<NarrowPolygonBox>;

class MarkAreaInfo {
 public:
  explicit MarkAreaInfo(const MarkArea &mark_area);

  const Id &id() const { return mark_area_.id(); }
  const Polygon2d &polygon() const { return polygon_; }
  const MarkArea markArea() const { return mark_area_; }
  bool inMarkArea(const Vec2d &point) const {
    return polygon_.IsPointIn(point);
  }
  double speed() const { return mark_area_.speed(); }
  bool hasSpeed() const { return mark_area_.has_speed(); }
  std::string material() const { return mark_area_.material(); }
  bool hasMaterial() const { return mark_area_.has_material(); }
  std::string color() const { return mark_area_.color(); }
  bool hasColor() const { return mark_area_.has_color(); }
  std::string getStatic() const { return mark_area_.static_area(); }
  bool hasStaticArea() const { return mark_area_.has_static_area(); }

 private:
  friend class hdMapImpl;
  void init();
  void postProcess(const HdMapImpl &map_instance);

 private:
  const MarkArea &mark_area_;
  Polygon2d polygon_;
  double speed_;
};
using MarkPolygonBox = ObjectWithAABox<MarkAreaInfo, Polygon2d>;
using MarkPolygonKDTree = AABoxKDTree2d<MarkPolygonBox>;

class ProhibitedAreaInfo {
 public:
  explicit ProhibitedAreaInfo(const ProhibitedArea &prohibited_area);

  const Id &id() const { return prohibited_area_.id(); }
  const Polygon2d &polygon() const { return polygon_; }
  const ProhibitedArea &prohibitedArea() const { return prohibited_area_; }
  bool inProhibitedArea(const Vec2d &point) const {
    return polygon_.IsPointIn(point);
  }

 private:
  friend class hdMapImpl;
  void init();
  void postProcess(const HdMapImpl &map_instance);

 private:
  const ProhibitedArea &prohibited_area_;
  Polygon2d polygon_;
};
using ProhibitedPolygonBox = ObjectWithAABox<ProhibitedAreaInfo, Polygon2d>;
using ProhibitedPolygonKDTree = AABoxKDTree2d<ProhibitedPolygonBox>;

class JunctionInfo {
 public:
  explicit JunctionInfo(const Junction &junction);

  const Id &id() const { return junction_.id(); }
  const Junction &junction() const { return junction_; }
  const Polygon2d &polygon() const { return polygon_; }

  const std::vector<Id> &OverlapStopSignIds() const {
    return overlap_stop_sign_ids_;
  }

 private:
  friend class HdMapImpl;
  void Init();
  void PostProcess(const HdMapImpl &map_instance);
  void UpdateOverlaps(const HdMapImpl &map_instance);

 private:
  const Junction &junction_;
  Polygon2d polygon_;

  std::vector<Id> overlap_stop_sign_ids_;
  std::vector<Id> overlap_ids_;
};
using JunctionPolygonBox = ObjectWithAABox<JunctionInfo, Polygon2d>;
using JunctionPolygonKDTree = AABoxKDTree2d<JunctionPolygonBox>;

class SignalInfo {
 public:
  explicit SignalInfo(const Signal &signal);

  const Id &id() const { return signal_.id(); }
  const Signal &signal() const { return signal_; }
  const std::vector<LineSegment2d> &segments() const { return segments_; }

 private:
  void Init();

 private:
  const Signal &signal_;
  std::vector<LineSegment2d> segments_;
};
using SignalSegmentBox = ObjectWithAABox<SignalInfo, LineSegment2d>;
using SignalSegmentKDTree = AABoxKDTree2d<SignalSegmentBox>;

class CrosswalkInfo {
 public:
  explicit CrosswalkInfo(const Crosswalk &crosswalk);

  const Id &id() const { return crosswalk_.id(); }
  const Crosswalk &crosswalk() const { return crosswalk_; }
  const Polygon2d &polygon() const { return polygon_; }

 private:
  void Init();

 private:
  const Crosswalk &crosswalk_;
  Polygon2d polygon_;
};
using CrosswalkPolygonBox = ObjectWithAABox<CrosswalkInfo, Polygon2d>;
using CrosswalkPolygonKDTree = AABoxKDTree2d<CrosswalkPolygonBox>;

class StopSignInfo {
 public:
  explicit StopSignInfo(const StopSign &stop_sign);

  const Id &id() const { return stop_sign_.id(); }
  const StopSign &stop_sign() const { return stop_sign_; }
  const std::vector<LineSegment2d> &segments() const { return segments_; }
  const std::vector<Id> &OverlapLaneIds() const { return overlap_lane_ids_; }
  const std::vector<Id> &OverlapJunctionIds() const {
    return overlap_junction_ids_;
  }

 private:
  friend class HdMapImpl;
  void init();
  void PostProcess(const HdMapImpl &map_instance);
  void UpdateOverlaps(const HdMapImpl &map_instance);

 private:
  const StopSign &stop_sign_;
  std::vector<LineSegment2d> segments_;

  std::vector<Id> overlap_lane_ids_;
  std::vector<Id> overlap_junction_ids_;
  std::vector<Id> overlap_ids_;
};
using StopSignSegmentBox = ObjectWithAABox<StopSignInfo, LineSegment2d>;
using StopSignSegmentKDTree = AABoxKDTree2d<StopSignSegmentBox>;

class YieldSignInfo {
 public:
  explicit YieldSignInfo(const YieldSign &yield_sign);

  const Id &id() const { return yield_sign_.id(); }
  const YieldSign &yield_sign() const { return yield_sign_; }
  const std::vector<LineSegment2d> &segments() const { return segments_; }

 private:
  void Init();

 private:
  const YieldSign &yield_sign_;
  std::vector<LineSegment2d> segments_;
};
using YieldSignSegmentBox = ObjectWithAABox<YieldSignInfo, LineSegment2d>;
using YieldSignSegmentKDTree = AABoxKDTree2d<YieldSignSegmentBox>;

class ClearAreaInfo {
 public:
  explicit ClearAreaInfo(const ClearArea &clear_area);

  const Id &id() const { return clear_area_.id(); }
  const ClearArea &clear_area() const { return clear_area_; }
  const Polygon2d &polygon() const { return polygon_; }

 private:
  void Init();

 private:
  const ClearArea &clear_area_;
  Polygon2d polygon_;
};
using ClearAreaPolygonBox = ObjectWithAABox<ClearAreaInfo, Polygon2d>;
using ClearAreaPolygonKDTree = AABoxKDTree2d<ClearAreaPolygonBox>;

class SpeedBumpInfo {
 public:
  explicit SpeedBumpInfo(const SpeedBump &speed_bump);

  const Id &id() const { return speed_bump_.id(); }
  double speed() const { return speed_bump_.speed(); }
  const SpeedBump &speed_bump() const { return speed_bump_; }
  const Polygon2d &polygon() const { return polygon_; }
  bool hasMaterial() const { return speed_bump_.has_material(); }
  std::string material() const { return speed_bump_.material(); }
  bool hasColor() const { return speed_bump_.has_color(); }
  std::string color() const { return speed_bump_.color(); }
  bool hasStaticArea() const { return speed_bump_.has_static_area(); }
  std::string staticArea() const { return speed_bump_.static_area(); }
  bool inSpeedBump(const Vec2d &point) const {
    return polygon_.IsPointIn(point);
  }
  const std::vector<LineSegment2d> &segments() const { return segments_; }

 private:
  void Init();

 private:
  const SpeedBump &speed_bump_;
  std::vector<LineSegment2d> segments_;
  Polygon2d polygon_;
};
using SpeedBumpSegmentBox = ObjectWithAABox<SpeedBumpInfo, LineSegment2d>;
using SpeedBumpSegmentKDTree = AABoxKDTree2d<SpeedBumpSegmentBox>;

class BlackAreaInfo {
 public:
  explicit BlackAreaInfo(const BlackArea &black_area);

  const Id &id() const { return black_area_.id(); }
  const BlackArea &black_area() const { return black_area_; }
  const Polygon2d &polygon() const { return polygon_; }
  bool inBlackArea(const Vec2d &point) const {
    return polygon_.IsPointIn(point);
  }

 private:
  void Init();

 private:
  const BlackArea &black_area_;
  Polygon2d polygon_;
};

using BlackAreaPolygonBox = ObjectWithAABox<BlackAreaInfo, Polygon2d>;
using BlackAreaKDTree = AABoxKDTree2d<BlackAreaPolygonBox>;

class OverlapInfo {
 public:
  explicit OverlapInfo(const Overlap &overlap);

  const Id &id() const { return overlap_.id(); }
  const Overlap &overlap() const { return overlap_; }
  const ObjectOverlapInfo *GetObjectOverlapInfo(const Id &id) const;

 private:
  const Overlap &overlap_;
};

class RoadInfo {
 public:
  explicit RoadInfo(const Road &road);
  const Id &id() const { return road_.id(); }
  const Road &road() const { return road_; }
  const std::vector<RoadSection> &sections() const { return sections_; }

  const Id &junction_id() const { return road_.junction_id(); }
  bool has_junction_id() const { return road_.has_junction_id(); }

  const std::vector<RoadBoundary> &GetBoundaries() const;

  cvte::hdmap::Road_Type type() const { return road_.type(); }

 private:
  Road road_;
  std::vector<RoadSection> sections_;
  std::vector<RoadBoundary> road_boundaries_;
};

class ParkingSpaceInfo {
 public:
  explicit ParkingSpaceInfo(const ParkingSpace &parkingspace);
  const Id &id() const { return parking_space_.id(); }
  const ParkingSpace &parking_space() const { return parking_space_; }
  const Polygon2d &polygon() const { return polygon_; }

 private:
  void Init();

 private:
  const ParkingSpace &parking_space_;
  Polygon2d polygon_;
};
using ParkingSpacePolygonBox = ObjectWithAABox<ParkingSpaceInfo, Polygon2d>;
using ParkingSpacePolygonKDTree = AABoxKDTree2d<ParkingSpacePolygonBox>;

class PNCJunctionInfo {
 public:
  explicit PNCJunctionInfo(const PNCJunction &pnc_junction);

  const Id &id() const { return junction_.id(); }
  const PNCJunction &pnc_junction() const { return junction_; }
  const Polygon2d &polygon() const { return polygon_; }

 private:
  void Init();

 private:
  const PNCJunction &junction_;
  Polygon2d polygon_;

  std::vector<Id> overlap_ids_;
};
using PNCJunctionPolygonBox = ObjectWithAABox<PNCJunctionInfo, Polygon2d>;
using PNCJunctionPolygonKDTree = AABoxKDTree2d<PNCJunctionPolygonBox>;

struct JunctionBoundary {
  JunctionInfoConstPtr junction_info;
};

using JunctionBoundaryPtr = std::shared_ptr<JunctionBoundary>;
}  // namespace hdmap
}  // namespace cvte
#endif