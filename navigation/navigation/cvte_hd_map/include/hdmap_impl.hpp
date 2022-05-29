/**
 * @file hdmap_impl.hpp
 * @author linyanlong (linyanlong@cvte.com)
 * @brief
 * @version 0.1
 * @date 2021-12-13
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef HDMAP_IMPL_HPP_
#define HDMAP_IMPL_HPP_

#include <string>
#include <vector>
#include "aabox2d.h"
#include "map_c5_area.pb.h"
#include "aaboxkdtree2d.h"
#include "line_segment2d.h"
#include "polygon2d.h"
#include "hdmap_common.hpp"
#include "map.pb.h"
#include "map_clear_area.pb.h"
#include "map_crosswalk.pb.h"
#include "map_geometry.pb.h"
#include "map_junction.pb.h"
#include "map_lane.pb.h"
#include "map_overlap.pb.h"
#include "map_parking_space.pb.h"
#include "map_pnc_junction.pb.h"
#include "map_signal.pb.h"
#include "map_speed_bump.pb.h"
#include "map_stop_sign.pb.h"
#include "map_yield_sign.pb.h"
#include "map_c5_area.pb.h"

namespace cvte {
namespace hdmap {
class HdMapImpl {
 public:
  using LaneTable = std::unordered_map<std::string, std::shared_ptr<LaneInfo>>;
  using JunctionTable =
      std::unordered_map<std::string, std::shared_ptr<JunctionInfo>>;
  using SignalTable =
      std::unordered_map<std::string, std::shared_ptr<SignalInfo>>;
  using CrosswalkTable =
      std::unordered_map<std::string, std::shared_ptr<CrosswalkInfo>>;
  using StopSignTable =
      std::unordered_map<std::string, std::shared_ptr<StopSignInfo>>;
  using YieldSignTable =
      std::unordered_map<std::string, std::shared_ptr<YieldSignInfo>>;
  using ClearAreaTable =
      std::unordered_map<std::string, std::shared_ptr<ClearAreaInfo>>;
  using SpeedBumpTable =
      std::unordered_map<std::string, std::shared_ptr<SpeedBumpInfo>>;

  using BlackAreaTable =
      std::unordered_map<std::string, std::shared_ptr<BlackAreaInfo>>;
  using OverlapTable =
      std::unordered_map<std::string, std::shared_ptr<OverlapInfo>>;
  using RoadTable = std::unordered_map<std::string, std::shared_ptr<RoadInfo>>;
  using ParkingSpaceTable =
      std::unordered_map<std::string, std::shared_ptr<ParkingSpaceInfo>>;
  using PNCJunctionTable =
      std::unordered_map<std::string, std::shared_ptr<PNCJunctionInfo>>;
  //清洁任务相关
  using MapAreaTable =
      std::unordered_map<std::string, std::shared_ptr<MapAreaInfo>>;
  using CleanAreaTable =
      std::unordered_map<std::string, std::shared_ptr<CleanAreaInfo>>;
  using PitAreaTable =
      std::unordered_map<std::string, std::shared_ptr<PitAreaInfo>>;
  using SlopeAreaTable =
      std::unordered_map<std::string, std::shared_ptr<SlopeAreaInfo>>;
  using ElevatorTable =
      std::unordered_map<std::string, std::shared_ptr<ElevatorInfo>>;
  using NarrowAreaTable =
      std::unordered_map<std::string, std::shared_ptr<NarrowAreaInfo>>;
  using ProhibitedAreaTable =
      std::unordered_map<std::string, std::shared_ptr<ProhibitedAreaInfo>>;
  using MarkAreaTable =
      std::unordered_map<std::string, std::shared_ptr<MarkAreaInfo>>;

 public:
  /**
   * @brief load map from local file
   * @param map_filename path of map data file
   * @return 0:success, otherwise failed
   */
  int LoadMapFromFile(const std::string &map_filename);
  int LoadMapFromProto(const Map &map_proto);

  LaneInfoConstPtr GetLaneById(const Id &id) const;
  JunctionInfoConstPtr GetJunctionById(const Id &id) const;
  SignalInfoConstPtr GetSignalById(const Id &id) const;
  CrosswalkInfoConstPtr GetCrosswalkById(const Id &id) const;
  StopSignInfoConstPtr GetStopSignById(const Id &id) const;
  YieldSignInfoConstPtr GetYieldSignById(const Id &id) const;
  ClearAreaInfoConstPtr GetClearAreaById(const Id &id) const;
  SpeedBumpInfoConstPtr GetSpeedBumpById(const Id &id) const;
  BlackAreaInfoConstPtr GetBlackAreaById(const Id &id) const;
  OverlapInfoConstPtr GetOverlapById(const Id &id) const;
  RoadInfoConstPtr GetRoadById(const Id &id) const;
  ParkingSpaceInfoConstPtr GetParkingSpaceById(const Id &id) const;
  PNCJunctionInfoConstPtr GetPNCJunctionById(const Id &id) const;
  MapAreaInfoConstPtr GetMapAreaById(const Id &id) const;
  CleanAreaInfoConstPtr GetCleanAreaById(const Id &id) const;
  PitAreaInfoConstPtr GetPitAreaById(const Id &id) const;
  SlopeAreaInfoConstPtr GetSlopeAreaById(const Id &id) const;
  ElevatorInfoConstPtr GetElevatorAreaById(const Id &id) const;
  NarrowAreaInfoConstPtr GetNarrowAreaById(const Id &id) const;
  ProhibitedAreaInfoConstPtr GetProhibitedAreaById(const Id &id) const;
  MarkAreaInfoConstPtr GetMarkAreaById(const Id &id) const;

  /**
   * @brief get all lanes in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param lanes store all lanes in target range
   * @return 0:success, otherwise failed
   */
  int GetLanes(const Vec2d &point, double distance,
               std::vector<LaneInfoConstPtr> *lanes) const;
  /**
   * @brief get all junctions in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param junctions store all junctions in target range
   * @return 0:success, otherwise failed
   */
  int GetJunctions(const Vec2d &point, double distance,
                   std::vector<JunctionInfoConstPtr> *junctions) const;
  /**
   * @brief get all crosswalks in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param crosswalks store all crosswalks in target range
   * @return 0:success, otherwise failed
   */
  int GetCrosswalks(const Vec2d &point, double distance,
                    std::vector<CrosswalkInfoConstPtr> *crosswalks) const;
  /**
   * @brief get all signals in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param signals store all signals in target range
   * @return 0:success, otherwise failed
   */
  int GetSignals(const Vec2d &point, double distance,
                 std::vector<SignalInfoConstPtr> *signals) const;
  /**
   * @brief get all stop signs in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param stop signs store all stop signs in target range
   * @return 0:success, otherwise failed
   */
  int GetStopSigns(const Vec2d &point, double distance,
                   std::vector<StopSignInfoConstPtr> *stop_signs) const;
  /**
   * @brief get all yield signs in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param yield signs store all yield signs in target range
   * @return 0:success, otherwise failed
   */
  int GetYieldSigns(const Vec2d &point, double distance,
                    std::vector<YieldSignInfoConstPtr> *yield_signs) const;
  /**
   * @brief get all clear areas in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param clear_areas store all clear areas in target range
   * @return 0:success, otherwise failed
   */
  int GetClearAreas(const Vec2d &point, double distance,
                    std::vector<ClearAreaInfoConstPtr> *clear_areas) const;
  /**
   * @brief get all speed bumps in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param speed_bumps store all speed bumps in target range
   * @return 0:success, otherwise failed
   */
  int GetSpeedBumps(const Vec2d &point, double distance,
                    std::vector<SpeedBumpInfoConstPtr> *speed_bumps) const;

  /**
   * @brief Get the Black Areas object
   *
   * @param point
   * @param distance
   * @param black_area
   * @return int
   */
  int GetBlackAreas(const Vec2d &point, double distance,
                    std::vector<BlackAreaInfoConstPtr> *black_area) const;
  /**
   * @brief get all roads in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param roads store all roads in target range
   * @return 0:success, otherwise failed
   */
  int GetRoads(const Vec2d &point, double distance,
               std::vector<RoadInfoConstPtr> *roads) const;

  /**
   * @brief get all parking space in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param parking_spaces store all parking spaces in target range
   * @return 0:success, otherwise failed
   */
  int GetParkingSpaces(
      const Vec2d &point, double distance,
      std::vector<ParkingSpaceInfoConstPtr> *parking_spaces) const;

  /**
   * @brief get all pnc junctions in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param junctions store all junctions in target range
   * @return 0:success, otherwise failed
   */
  int GetPNCJunctions(
      const Vec2d &point, double distance,
      std::vector<PNCJunctionInfoConstPtr> *pnc_junctions) const;

  /**
   * @brief get nearest lane from target point,
   * @param point the target point
   * @param nearest_lane the nearest lane that match search conditions
   * @param nearest_s the offset from lane start point along lane center line
   * @param nearest_l the lateral offset from lane center line
   * @return 0:success, otherwise, failed.
   */
  int GetNearestLane(const Vec2d &point, LaneInfoConstPtr *nearest_lane,
                     double *nearest_s, double *nearest_l) const;
  /**
   * @brief get the nearest lane within a certain range by pose
   * @param point the target position
   * @param distance the search radius
   * @param central_heading the base heading
   * @param max_heading_difference the heading range
   * @param nearest_lane the nearest lane that match search conditions
   * @param nearest_s the offset from lane start point along lane center line
   * @param nearest_l the lateral offset from lane center line
   * @return 0:success, otherwise, failed.
   */
  int GetNearestLaneWithHeading(const Vec2d &point, const double distance,
                                const double central_heading,
                                const double max_heading_difference,
                                LaneInfoConstPtr *nearest_lane,
                                double *nearest_s, double *nearest_l) const;
  /**
   * @brief get all lanes within a certain range by pose
   * @param point the target position
   * @param distance the search radius
   * @param central_heading the base heading
   * @param max_heading_difference the heading range
   * @param nearest_lane all lanes that match search conditions
   * @return 0:success, otherwise, failed.
   */
  int GetLanesWithHeading(const Vec2d &point, const double distance,
                          const double central_heading,
                          const double max_heading_difference,
                          std::vector<LaneInfoConstPtr> *lanes) const;
  /**
   * @brief get all road and junctions boundaries within certain range
   * @param point the target position
   * @param radius the search radius
   * @param road_boundaries the roads' boundaries
   * @param junctions the junctions' boundaries
   * @return 0:success, otherwise failed
   */
  int GetRoadBoundaries(const Vec2d &point, double radius,
                        std::vector<RoadROIBoundaryPtr> *road_boundaries,
                        std::vector<JunctionBoundaryPtr> *junctions) const;
  /**
   * @brief get all road boundaries and junctions within certain range
   * @param point the target position
   * @param radius the search radius
   * @param road_boundaries the roads' boundaries
   * @param junctions the junctions
   * @return 0:success, otherwise failed
   */
  int GetRoadBoundaries(const Vec2d &point, double radius,
                        std::vector<RoadRoiPtr> *road_boundaries,
                        std::vector<JunctionInfoConstPtr> *junctions) const;
  /**
   * @brief get ROI within certain range
   * @param point the target position
   * @param radius the search radius
   * @param roads_roi the roads' boundaries
   * @param polygons_roi the junctions' boundaries
   * @return 0:success, otherwise failed
   */
  int GetRoi(const Vec2d &point, double radius,
             std::vector<RoadRoiPtr> *roads_roi,
             std::vector<PolygonRoiPtr> *polygons_roi) const;

  int GetSlopeArea(const Vec2d &point, double radius,
                   std::vector<SlopeAreaInfoConstPtr> *slope_areas) const;

  int GetPitArea(const Vec2d &point, double radius,
                 std::vector<PitAreaInfoConstPtr> *pit_areas) const;

  int GetCleanArea(const Vec2d &point, double radius,
                   std::vector<CleanAreaInfoConstPtr> *clean_areas) const;

  int GetNarrowArea(const Vec2d &point, double radius,
                    std::vector<NarrowAreaInfoConstPtr> *narrow_areas) const;

  int GetElevatorArea(const Vec2d &point, double radius,
                      std::vector<ElevatorInfoConstPtr> *elevator_areas) const;

  int GetMarkArea(const Vec2d &point, double radius,
                  std::vector<MarkAreaInfoConstPtr> *mark_areas) const;

  int GetProhibitedArea(
      const Vec2d &point, double radius,
      std::vector<ProhibitedAreaInfoConstPtr> *prohibited_areas) const;

  /**
   * @brief get forward nearest signals within certain range on the lane
   *        if there are two signals related to one stop line,
   *        return both signals.
   * @param point the target position
   * @param distance the forward search distance
   * @param signals all signals match conditions
   * @return 0:success, otherwise failed
   */
  int GetForwardNearestSignalsOnLane(
      const Vec2d &point, const double distance,
      std::vector<SignalInfoConstPtr> *signals) const;

  /**
   * @brief get all other stop signs associated with a stop sign
   *        in the same junction
   * @param id id of stop sign
   * @param stop_signs stop signs associated
   * @return 0:success, otherwise failed
   */
  int GetStopSignAssociatedStopSigns(
      const Id &id, std::vector<StopSignInfoConstPtr> *stop_signs) const;

  /**
   * @brief get all lanes associated with a stop sign in the same junction
   * @param id id of stop sign
   * @param lanes all lanes match conditions
   * @return 0:success, otherwise failed
   */
  int GetStopSignAssociatedLanes(const Id &id,
                                 std::vector<LaneInfoConstPtr> *lanes) const;

  /**
   * @brief get a local map which is identical to the origin map except that all
   * map elements without overlap with the given region are deleted.
   * @param point the target position
   * @param range the size of local map region, [width, height]
   * @param local_map local map in proto format
   * @return 0:success, otherwise failed
   */
  int GetLocalMap(const Vec2d &point, const std::pair<double, double> &range,
                  Map *local_map) const;

 private:
  template <class Table, class BoxTable, class KDTree>
  static void BuildSegmentKDTree(const Table &table,
                                 const AABoxKDTreeParams &params,
                                 BoxTable *const box_table,
                                 std::unique_ptr<KDTree> *const kdtree);

  template <class Table, class BoxTable, class KDTree>
  static void BuildPolygonKDTree(const Table &table,
                                 const AABoxKDTreeParams &params,
                                 BoxTable *const box_table,
                                 std::unique_ptr<KDTree> *const kdtree);

  void BuildLaneSegmentKDTree();
  void BuildJunctionPolygonKDTree();
  void BuildCrosswalkPolygonKDTree();
  void BuildSignalSegmentKDTree();
  void BuildStopSignSegmentKDTree();
  void BuildYieldSignSegmentKDTree();
  void BuildClearAreaPolygonKDTree();
  void BuildSpeedBumpSegmentKDTree();
  void BuildParkingSpacePolygonKDTree();
  void BuildPNCJunctionPolygonKDTree();
  //   void BuildMapAreaPolygonKDTree();
  void BuildCleanAreaPolygonKDTree();
  void BuildSlopeAreaPolygonKDTree();
  void BuildPitAreaPolygonKDTree();
  void BuildNarrowAreaPolygonKDTree();
  void BuildElevatorPolygonKDTree();
  void BuildMarkPolygonKDTree();
  void BuildProhibitedPolygonKDTree();
  void BuildBlackAreaPolygonKDTree();

  template <class KDTree>
  static int SearchObjects(const math_utils::Vec2d &center, const double radius,
                           const KDTree &kdtree,
                           std::vector<std::string> *const results);

  void Clear();

 private:
  Map map_;
  LaneTable lane_table_;
  JunctionTable junction_table_;
  CrosswalkTable crosswalk_table_;
  SignalTable signal_table_;
  StopSignTable stop_sign_table_;
  YieldSignTable yield_sign_table_;
  ClearAreaTable clear_area_table_;
  SpeedBumpTable speed_bump_table_;
  BlackAreaTable black_area_table_;
  OverlapTable overlap_table_;
  RoadTable road_table_;
  ParkingSpaceTable parking_space_table_;
  PNCJunctionTable pnc_junction_table_;
  MapAreaTable map_area_table_;
  PitAreaTable pit_area_table_;
  SlopeAreaTable slope_area_table_;
  CleanAreaTable clean_area_table_;
  NarrowAreaTable narrow_area_table_;
  ElevatorTable elevator_table_;
  MarkAreaTable mark_area_table_;
  ProhibitedAreaTable prohibited_area_table_;

  std::vector<LaneSegmentBox> lane_segment_boxes_;
  std::unique_ptr<LaneSegmentKDTree> lane_segment_kdtree_;

  std::vector<JunctionPolygonBox> junction_polygon_boxes_;
  std::unique_ptr<JunctionPolygonKDTree> junction_polygon_kdtree_;

  std::vector<CrosswalkPolygonBox> crosswalk_polygon_boxes_;
  std::unique_ptr<CrosswalkPolygonKDTree> crosswalk_polygon_kdtree_;

  std::vector<SignalSegmentBox> signal_segment_boxes_;
  std::unique_ptr<SignalSegmentKDTree> signal_segment_kdtree_;

  std::vector<StopSignSegmentBox> stop_sign_segment_boxes_;
  std::unique_ptr<StopSignSegmentKDTree> stop_sign_segment_kdtree_;

  std::vector<YieldSignSegmentBox> yield_sign_segment_boxes_;
  std::unique_ptr<YieldSignSegmentKDTree> yield_sign_segment_kdtree_;

  std::vector<ClearAreaPolygonBox> clear_area_polygon_boxes_;
  std::unique_ptr<ClearAreaPolygonKDTree> clear_area_polygon_kdtree_;

  std::vector<SpeedBumpSegmentBox> speed_bump_segment_boxes_;
  std::unique_ptr<SpeedBumpSegmentKDTree> speed_bump_segment_kdtree_;

  std::vector<BlackAreaPolygonBox> black_area_polygon_boxes_;
  std::unique_ptr<BlackAreaKDTree> black_area_polygon_kdtree_;

  std::vector<ParkingSpacePolygonBox> parking_space_polygon_boxes_;
  std::unique_ptr<ParkingSpacePolygonKDTree> parking_space_polygon_kdtree_;

  std::vector<PNCJunctionPolygonBox> pnc_junction_polygon_boxes_;
  std::unique_ptr<PNCJunctionPolygonKDTree> pnc_junction_polygon_kdtree_;

  std::vector<MapPolygonBox> map_polygon_boxes_;
  std::unique_ptr<MapPolygonKDTree> map_polygon_kdtree_;

  std::vector<CleanPolygonBox> clean_polygon_boxes_;
  std::unique_ptr<CleanPolygonKDTree> clean_polygon_kdtree_;

  std::vector<SlopePolygonBox> slope_polygon_boxed_;
  std::unique_ptr<SlopePolygonKDTree> slope_polygon_kdtree_;

  std::vector<PitPolygonBox> pit_polygon_boxed_;
  std::unique_ptr<PitPolygonKDTree> pit_polygon_kdtree_;

  std::vector<NarrowPolygonBox> narrow_polygon_boxed_;
  std::unique_ptr<NarrowPolygonKDTree> narrow_polygon_kdtree_;

  std::vector<ElevatorPolygonBox> elevator_polygon_boxed_;
  std::unique_ptr<ElevatorPolygonKDTree> elevator_polygon_kdtree_;

  std::vector<MarkPolygonBox> mark_polygon_boxed_;
  std::unique_ptr<MarkPolygonKDTree> mark_polygon_kdtree_;

  std::vector<ProhibitedPolygonBox> prohibited_polygon_boxed_;
  std::unique_ptr<ProhibitedPolygonKDTree> prohibited_polygon_kdtree_;
};
}  // namespace hdmap
}  // namespace cvte
#endif