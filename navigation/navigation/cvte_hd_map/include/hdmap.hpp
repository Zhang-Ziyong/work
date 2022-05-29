/**
 * @file hdmap.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-01-05
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef HDMAP_HPP_
#define HDMAP_HPP_
#include "hdmap_impl.hpp"
#include "hdmap_common.hpp"
namespace cvte {
namespace hdmap {

class CvteHdMap {
 public:
  /**
   * @brief 获取单例指针
   *
   * @return std::shared_ptr<CvteHdMap>
   */
  static std::shared_ptr<CvteHdMap> getInstance();

  /**
   * @brief load map from local file
   * @param map_filename path of map data file
   * @return 0:success, otherwise failed
   */
  int LoadMapFromFile(const std::string &map_filename);

  Id GetCurrentMapId() const;

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
   * @param black_areas
   * @return int
   */
  int GetBlackAreas(const Vec2d &point, double distance,
                    std::vector<BlackAreaInfoConstPtr> *black_areas) const;
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
  CvteHdMap() { map_id_.set_id(""); }
  static std::shared_ptr<CvteHdMap> ptr_cvte_hdmap_;
  HdMapImpl hdmap_impl_;
  Id map_id_;
};

}  // namespace hdmap
}  // namespace cvte

#endif