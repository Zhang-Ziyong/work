
#include "gtest/gtest.h"
#include "hdmap_impl.hpp"
#include "hdmap_common.hpp"

namespace cvte {
namespace hdmap {

class HDMapCommonTestSuite : public ::testing::Test {
 protected:
  HDMapCommonTestSuite() {}
  virtual ~HDMapCommonTestSuite() {}
  virtual void SetUp() {}
  virtual void TearDown() {}
  void InitLaneObj(Lane *lane);
  void InitJunctionObj(Junction *junction);
  void InitSignalObj(Signal *signal);
  void InitCrosswalkObj(Crosswalk *crosswalk);
  void InitStopSignObj(StopSign *stop_sign);
  void InitYieldSignObj(YieldSign *yield_sign);
  void InitClearAreaObj(ClearArea *clear_area);
  void InitSpeedBumpObj(SpeedBump *speed_bump);
  void InitRoadObj(Road *road);
  void InitParkingSpaceObj(ParkingSpace *parking_space);

  void InitCleanAreaObj(CleanArea *clean_area);
  void InitSlopeAreaObj(SlopeArea *slope_area);
  void InitElevatorObj(Elevator *elevator);
  void InitNarrowAreaObj(NarrowArea *narrow_area);
  void InitMarkAreaObj(MarkArea *mark_area);
  void InitProhibitedObj(ProhibitedArea *prohibited_area);
  void InitMapAreaObj(MapArea *map_area);
};

void HDMapCommonTestSuite::InitLaneObj(Lane *lane) {
  lane->mutable_id()->set_id("lane_1");
  CurveSegment *curve_segment = lane->mutable_central_curve()->add_segment();
  LineSegment *line_segment = curve_segment->mutable_line_segment();
  Point2d *pt = line_segment->add_point();
  pt->set_x(170001.0);
  pt->set_y(1.0);
  pt = line_segment->add_point();
  pt->set_x(170002.0);
  pt->set_y(1.0);
  pt = line_segment->add_point();
  pt->set_x(170003.0);
  pt->set_y(1.0);
  pt = line_segment->add_point();
  pt->set_x(170004.0);
  pt->set_y(1.0);
  pt = line_segment->add_point();
  pt->set_x(170005.0);
  pt->set_y(1.0);
  LaneSampleAssociation *lane_sample = lane->add_left_sample();
  lane_sample->set_s(0.0);
  lane_sample->set_width(1.5);
  lane_sample = lane->add_left_sample();
  lane_sample->set_s(1.0);
  lane_sample->set_width(1.5);
  lane_sample = lane->add_left_sample();
  lane_sample->set_s(2.0);
  lane_sample->set_width(1.5);
  lane_sample = lane->add_left_sample();
  lane_sample->set_s(3.0);
  lane_sample->set_width(1.2);
  lane_sample = lane->add_left_sample();
  lane_sample->set_s(4.0);
  lane_sample->set_width(1.2);

  lane_sample = lane->add_right_sample();
  lane_sample->set_s(0.0);
  lane_sample->set_width(1.5);
  lane_sample = lane->add_right_sample();
  lane_sample->set_s(1.0);
  lane_sample->set_width(1.5);
  lane_sample = lane->add_right_sample();
  lane_sample->set_s(2.0);
  lane_sample->set_width(1.5);
  lane_sample = lane->add_right_sample();
  lane_sample->set_s(3.0);
  lane_sample->set_width(1.2);
  lane_sample = lane->add_right_sample();
  lane_sample->set_s(4.0);
  lane_sample->set_width(1.2);

  lane->set_type(Lane::CITY_DRIVING);
}

void HDMapCommonTestSuite::InitJunctionObj(Junction *junction) {
  junction->mutable_id()->set_id("junction_1");
  Polygon *polygon = junction->mutable_polygon();
  Point2d *pt = polygon->add_point();
  pt->set_x(170001.0);
  pt->set_y(170001.0);
  pt = polygon->add_point();
  pt->set_x(170002.0);
  pt->set_y(170001.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170001.0);
  pt = polygon->add_point();
  pt->set_x(170004.0);
  pt->set_y(170001.0);
  pt = polygon->add_point();
  pt->set_x(170005.0);
  pt->set_y(170001.0);
  pt = polygon->add_point();
  pt->set_x(170005.0);
  pt->set_y(170001.0);
  pt = polygon->add_point();
  pt->set_x(170005.0);
  pt->set_y(170002.0);
  pt = polygon->add_point();
  pt->set_x(170001.0);
  pt->set_y(170002.0);
  pt = polygon->add_point();
  pt->set_x(170001.0);
  pt->set_y(170001.0);
}

void HDMapCommonTestSuite::InitSignalObj(Signal *signal) {
  signal->mutable_id()->set_id("signal_1");
  Polygon *polygon = signal->mutable_boundary();
  Point2d *pt = polygon->add_point();
  pt->set_x(170001.0);
  pt->set_y(1.0);
  pt = polygon->add_point();
  pt->set_x(170001.0);
  pt->set_y(1.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(1.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(1.0);

  Subsignal *sub_signal = signal->add_subsignal();
  sub_signal->mutable_id()->set_id("sub_signal_1");
  pt = sub_signal->mutable_location();
  pt->set_x(170002.0);
  pt->set_y(1.0);
  sub_signal = signal->add_subsignal();
  sub_signal->mutable_id()->set_id("sub_signal_2");
  pt = sub_signal->mutable_location();
  pt->set_x(170002.0);
  pt->set_y(1.0);
  sub_signal = signal->add_subsignal();
  sub_signal->mutable_id()->set_id("sub_signal_3");
  pt = sub_signal->mutable_location();
  pt->set_x(170002.0);
  pt->set_y(1.0);

  CurveSegment *curve_segment = signal->add_stop_line()->add_segment();
  LineSegment *line_segment = curve_segment->mutable_line_segment();
  pt = line_segment->add_point();
  pt->set_x(170000.0);
  pt->set_y(4.0);
  pt = line_segment->add_point();
  pt->set_x(170001.0);
  pt->set_y(4.0);
  pt = line_segment->add_point();
  pt->set_x(170002.0);
  pt->set_y(4.0);

  curve_segment = signal->add_stop_line()->add_segment();
  line_segment = curve_segment->mutable_line_segment();
  pt = line_segment->add_point();
  pt->set_x(170002.0);
  pt->set_y(4.0);
  pt = line_segment->add_point();
  pt->set_x(170003.0);
  pt->set_y(4.0);
  pt = line_segment->add_point();
  pt->set_x(170004.0);
  pt->set_y(4.0);
}
void HDMapCommonTestSuite::InitCrosswalkObj(Crosswalk *crosswalk) {
  crosswalk->mutable_id()->set_id("crosswalk_1");
  Polygon *polygon = crosswalk->mutable_polygon();
  Point2d *pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170000.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170000.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170003.0);
  pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170003.0);
}
void HDMapCommonTestSuite::InitStopSignObj(StopSign *stop_sign) {
  stop_sign->mutable_id()->set_id("stop_sign_1");
  CurveSegment *curve_segment = stop_sign->add_stop_line()->add_segment();
  LineSegment *line_segment = curve_segment->mutable_line_segment();
  Point2d *pt = line_segment->add_point();
  pt->set_x(170000.0);
  pt->set_y(0.0);
  pt = line_segment->add_point();
  pt->set_x(170001.0);
  pt->set_y(0.0);
  pt = line_segment->add_point();
  pt->set_x(170002.0);
  pt->set_y(0.0);
}
void HDMapCommonTestSuite::InitYieldSignObj(YieldSign *yield_sign) {
  yield_sign->mutable_id()->set_id("yield_sign_1");
  CurveSegment *curve_segment = yield_sign->add_stop_line()->add_segment();
  LineSegment *line_segment = curve_segment->mutable_line_segment();
  Point2d *pt = line_segment->add_point();
  pt->set_x(170000.0);
  pt->set_y(0.0);
  pt = line_segment->add_point();
  pt->set_x(170001.0);
  pt->set_y(0.0);
  pt = line_segment->add_point();
  pt->set_x(170002.0);
  pt->set_y(0.0);
}
void HDMapCommonTestSuite::InitClearAreaObj(ClearArea *clear_area) {
  clear_area->mutable_id()->set_id("clear_area_1");
  Polygon *polygon = clear_area->mutable_polygon();
  Point2d *pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170000.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170000.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170003.0);
  pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170003.0);
}
void HDMapCommonTestSuite::InitSpeedBumpObj(SpeedBump *speed_bump) {
  speed_bump->mutable_id()->set_id("speed_bump_1");
  Polygon *polygon = speed_bump->mutable_polygon();
  CurveSegment *curve_segment = speed_bump->add_position()->add_segment();
  LineSegment *line_segment = curve_segment->mutable_line_segment();
  Point2d *pt = line_segment->add_point();
  pt->set_x(170000.0);
  pt->set_y(0.0);
  pt = line_segment->add_point();
  pt->set_x(170001.0);
  pt->set_y(0.0);
  pt = line_segment->add_point();
  pt->set_x(170002.0);
  pt->set_y(0.0);

  pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170000.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170000.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170003.0);
  pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170003.0);
  speed_bump->set_speed(1.0);
}
void HDMapCommonTestSuite::InitParkingSpaceObj(ParkingSpace *parking_space) {
  parking_space->mutable_id()->set_id("parking_space_1");
  Polygon *polygon = parking_space->mutable_polygon();
  Point2d *pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170000.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170000.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170003.0);
  pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170003.0);
}

void HDMapCommonTestSuite::InitRoadObj(Road *road) {
  road->mutable_id()->set_id("road_1");
  road->mutable_junction_id()->set_id("junction_1");

  RoadSection *section = road->add_section();
  section->mutable_id()->set_id("section_1");
  section->add_lane_id()->set_id("section_1_1");
  section->add_lane_id()->set_id("section_1_2");

  section = road->add_section();
  section->mutable_id()->set_id("section_2");
  section->add_lane_id()->set_id("section_2_1");
}

void HDMapCommonTestSuite::InitCleanAreaObj(CleanArea *clean_area) {
  clean_area->mutable_id()->set_id("clean_area_1");
  clean_area->set_speed(1.6);
  Polygon *polygon = clean_area->mutable_polygon();
  Point2d *pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170000.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170000.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170003.0);
  pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170003.0);
}

void HDMapCommonTestSuite::InitSlopeAreaObj(SlopeArea *slope_area) {
  slope_area->mutable_id()->set_id("slope_area_1");
  slope_area->set_angle(0.08);
  slope_area->set_length(2.0);
  slope_area->set_speed(0.2);
  Polygon *polygon = slope_area->mutable_polygon();
  Point2d *pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170000.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170000.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170003.0);
  pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170003.0);
}

void HDMapCommonTestSuite::InitElevatorObj(Elevator *elevator) {
  elevator->mutable_id()->set_id("elevator_1");
  Polygon *polygon = elevator->mutable_polygon();
  Point2d *pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170000.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170000.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170003.0);
  pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170003.0);
}

void HDMapCommonTestSuite::InitNarrowAreaObj(NarrowArea *narrow_area) {
  narrow_area->mutable_id()->set_id("narrow_area_1");
  narrow_area->set_length(1.0);
  narrow_area->set_width(1.0);
  Polygon *polygon = narrow_area->mutable_polygon();
  Point2d *pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170000.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170000.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170003.0);
  pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170003.0);
}

void HDMapCommonTestSuite::InitMarkAreaObj(MarkArea *mark_area) {
  mark_area->mutable_id()->set_id("mark_area_1");
  mark_area->set_speed(0.6);
  mark_area->set_material("mark");
  Polygon *polygon = mark_area->mutable_polygon();
  Point2d *pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170000.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170000.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170003.0);
  pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170003.0);
}

void HDMapCommonTestSuite::InitProhibitedObj(ProhibitedArea *prohibited_area) {
  prohibited_area->mutable_id()->set_id("prohibited_area_1");
  prohibited_area->set_reason("dangerous");
  Polygon *polygon = prohibited_area->mutable_polygon();
  Point2d *pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170000.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170000.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170003.0);
  pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170003.0);
}

void HDMapCommonTestSuite::InitMapAreaObj(MapArea *map_area) {
  map_area->mutable_id()->set_id("map_area_1");
  Polygon *polygon = map_area->mutable_polygon();
  Point2d *pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170000.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170000.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170003.0);
  pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170003.0);
}

TEST_F(HDMapCommonTestSuite, TestLaneInfo) {
  Lane lane;
  InitLaneObj(&lane);
  LaneInfo lane_info(lane);
  EXPECT_EQ(lane.id().id(), lane_info.id().id());
  EXPECT_EQ(lane.central_curve().segment(0).line_segment().point_size(),
            lane_info.points().size());
  for (int i = 0; i < static_cast<int>(lane_info.points().size()); ++i) {
    EXPECT_NEAR(lane.central_curve().segment(0).line_segment().point(i).x(),
                lane_info.points()[i].x(), 1E-5);
    EXPECT_NEAR(lane.central_curve().segment(0).line_segment().point(i).y(),
                lane_info.points()[i].y(), 1E-5);
  }
  EXPECT_EQ(lane.central_curve().segment(0).line_segment().point_size() - 1,
            lane_info.segments().size());
  for (const auto &segment : lane_info.segments()) {
    EXPECT_NEAR(1.0, segment.length(), 1E-4);
  }
  EXPECT_EQ(lane_info.unit_directions().size(),
            lane_info.segments().size() + 1);
  for (size_t i = 0; i < lane_info.segments().size(); ++i) {
    EXPECT_EQ(lane_info.segments()[i].unit_direction(),
              lane_info.unit_directions()[i]);
  }
  EXPECT_EQ(lane.central_curve().segment(0).line_segment().point_size(),
            lane_info.accumulate_s().size());
  for (size_t i = 0; i < lane_info.accumulate_s().size(); ++i) {
    EXPECT_NEAR(static_cast<double>(i) * 1.0,
                lane_info.accumulate_s()[static_cast<int>(i)], 1E-4);
  }
  EXPECT_EQ(lane.central_curve().segment(0).line_segment().point_size(),
            lane_info.headings().size());
  for (size_t i = 0; i < lane_info.headings().size(); ++i) {
    EXPECT_NEAR(atan2(lane_info.unit_directions()[i].y(),
                      lane_info.unit_directions()[i].x()),
                lane_info.headings()[i], 1E-3);
  }
  double left_width = 0.0;
  double right_width = 0.0;
  lane_info.GetWidth(2.0, &left_width, &right_width);
  EXPECT_NEAR(1.5, left_width, 1E-3);
  EXPECT_NEAR(1.5, right_width, 1E-3);
  lane_info.GetWidth(3.5, &left_width, &right_width);
  EXPECT_NEAR(1.2, left_width, 1E-3);
  EXPECT_NEAR(1.2, right_width, 1E-3);
  EXPECT_NEAR(4.0, lane_info.total_length(), 1E-3);
}

TEST_F(HDMapCommonTestSuite, GetWidth) {
  Lane lane;
  InitLaneObj(&lane);
  LaneInfo lane_info(lane);
  EXPECT_NEAR(3.0, lane_info.GetWidth(2.0), 1E-3);
  EXPECT_NEAR(2.4, lane_info.GetWidth(3.5), 1E-3);
}

TEST_F(HDMapCommonTestSuite, GetEffectiveWidth) {
  Lane lane;
  InitLaneObj(&lane);
  LaneInfo lane_info(lane);
  EXPECT_NEAR(3.0, lane_info.GetEffectiveWidth(2.0), 1E-3);
  EXPECT_NEAR(2.4, lane_info.GetEffectiveWidth(3.5), 1E-3);
}

TEST_F(HDMapCommonTestSuite, PointIsOnLane) {
  Lane lane;
  InitLaneObj(&lane);
  LaneInfo lane_info(lane);

  EXPECT_TRUE(lane_info.IsOnLane({170001.5, 1.5}));
  EXPECT_TRUE(lane_info.IsOnLane({170001.5, 0.5}));
  EXPECT_FALSE(lane_info.IsOnLane({170000.5, 1.5}));
  EXPECT_FALSE(lane_info.IsOnLane({170001.5, 3}));
}

TEST_F(HDMapCommonTestSuite, BoxIsOnLane) {
  Lane lane;
  InitLaneObj(&lane);
  LaneInfo lane_info(lane);

  Box2d target_in_box(LineSegment2d({170002, 1}, {170003, 1}), 0.5);
  EXPECT_TRUE(lane_info.IsOnLane(target_in_box));

  Box2d target_out_box(LineSegment2d({170002, 1}, {170003, 1}), 4);
  EXPECT_FALSE(lane_info.IsOnLane(target_out_box));
}

TEST_F(HDMapCommonTestSuite, GetSmoothPoint) {
  Lane lane;
  InitLaneObj(&lane);
  LaneInfo lane_info(lane);

  auto smooth_point = lane_info.GetSmoothPoint(1.5);
  EXPECT_NEAR(smooth_point.x(), 170002.5, 1E-3);
  EXPECT_NEAR(smooth_point.y(), 1.0, 1E-3);
}

TEST_F(HDMapCommonTestSuite, DistanceTo) {
  Lane lane;
  InitLaneObj(&lane);
  LaneInfo lane_info(lane);

  double distance = lane_info.DistanceTo({170002.5, 3.0});
  EXPECT_NEAR(distance, 2.0, 1E-3);

  distance = lane_info.DistanceTo({170000.5, 3.0});
  EXPECT_NEAR(distance, 2.0615, 1E-3);
}

TEST_F(HDMapCommonTestSuite, DistanceToWithMoreInfo) {
  Lane lane;
  InitLaneObj(&lane);
  LaneInfo lane_info(lane);

  Vec2d foot_point;
  double s_offset = 0.0;
  int s_offset_index = 0;
  double distance = lane_info.DistanceTo({170002.5, 3.0}, &foot_point,
                                         &s_offset, &s_offset_index);
  EXPECT_NEAR(distance, 2.0, 1E-3);
  EXPECT_NEAR(foot_point.x(), 170002.5, 1E-3);
  EXPECT_NEAR(foot_point.y(), 1.0, 1E-3);
  EXPECT_NEAR(s_offset, 1.5, 1E-3);

  distance = lane_info.DistanceTo({170000.5, 3.0}, &foot_point, &s_offset,
                                  &s_offset_index);
  EXPECT_NEAR(distance, 2.06155, 1E-3);
  EXPECT_NEAR(foot_point.x(), 170001.0, 1E-3);
  EXPECT_NEAR(foot_point.y(), 1.0, 1E-3);
  EXPECT_NEAR(s_offset, 0.0, 1E-3);
}

TEST_F(HDMapCommonTestSuite, GetNearestPoint) {
  Lane lane;
  InitLaneObj(&lane);
  LaneInfo lane_info(lane);

  double distance = 0.0;
  auto nearest_point = lane_info.GetNearestPoint({170002.4, 3.0}, &distance);
  EXPECT_NEAR(nearest_point.x(), 170002.4, 1E-3);
  EXPECT_NEAR(nearest_point.y(), 1.0, 1E-3);

  nearest_point = lane_info.GetNearestPoint({170000.5, 3.0}, &distance);
  EXPECT_NEAR(nearest_point.x(), 170001.0, 1E-3);
  EXPECT_NEAR(nearest_point.y(), 1.0, 1E-3);

  nearest_point = lane_info.GetNearestPoint({170010.5, 3.0}, &distance);
  EXPECT_NEAR(nearest_point.x(), 170005.0, 1E-3);
  EXPECT_NEAR(nearest_point.y(), 1.0, 1E-3);
}

TEST_F(HDMapCommonTestSuite, GetProjection) {
  Lane lane;
  InitLaneObj(&lane);
  LaneInfo lane_info(lane);

  double accumulate_s = 0.0;
  double lateral = 0.0;
  bool success =
      lane_info.GetProjection({170002.4, 3.0}, &accumulate_s, &lateral);
  EXPECT_TRUE(success);
  EXPECT_NEAR(accumulate_s, 1.4, 1E-3);
  EXPECT_NEAR(lateral, 2.0, 1E-3);

  success = lane_info.GetProjection({170000.5, 3.0}, &accumulate_s, &lateral);
  EXPECT_TRUE(success);
  EXPECT_NEAR(accumulate_s, -0.5, 1E-3);
  EXPECT_NEAR(lateral, 2.0, 1E-3);

  success = lane_info.GetProjection({170010.5, 3.0}, &accumulate_s, &lateral);
  EXPECT_TRUE(success);
  EXPECT_NEAR(accumulate_s, 9.5, 1E-3);
  EXPECT_NEAR(lateral, 2.0, 1E-3);
}

TEST_F(HDMapCommonTestSuite, TestJunctionInfo) {
  Junction junction;
  InitJunctionObj(&junction);
  JunctionInfo junction_info(junction);
  EXPECT_EQ(junction.id().id(), junction_info.id().id());
  EXPECT_EQ(7, junction_info.polygon().points().size());
  for (size_t i = 0; i < 5; ++i) {
    EXPECT_NEAR(static_cast<double>(i + 170001) * 1.0,
                junction_info.polygon().points()[i].x(), 1E-3);
  }
  EXPECT_NEAR(170005.0, junction_info.polygon().points()[5].x(), 1E-3);
  EXPECT_NEAR(170002.0, junction_info.polygon().points()[5].y(), 1E-3);
  EXPECT_NEAR(170001.0, junction_info.polygon().points()[6].x(), 1E-3);
  EXPECT_NEAR(170002.0, junction_info.polygon().points()[6].y(), 1E-3);
}

TEST_F(HDMapCommonTestSuite, TestSignalInfo) {
  Signal signal;
  InitSignalObj(&signal);
  SignalInfo signal_info(signal);
  EXPECT_EQ(signal.id().id(), signal_info.id().id());
  EXPECT_EQ(4, signal_info.signal().boundary().point_size());

  int segment_size = 0;
  for (const auto &stop_line : signal.stop_line()) {
    segment_size += stop_line.segment(0).line_segment().point_size() - 1;
  }
  EXPECT_EQ(segment_size, signal_info.segments().size());
  for (const auto &segment : signal_info.segments()) {
    EXPECT_NEAR(1.0, segment.length(), 1E-4);
  }
}

TEST_F(HDMapCommonTestSuite, TestCrosswalkInfo) {
  Crosswalk crosswalk;
  InitCrosswalkObj(&crosswalk);
  CrosswalkInfo crosswalk_info(crosswalk);
  EXPECT_EQ(crosswalk.id().id(), crosswalk_info.id().id());
  EXPECT_EQ(4, crosswalk_info.crosswalk().polygon().point_size());
  EXPECT_NEAR(170000.0, crosswalk_info.polygon().points()[0].x(), 1E-3);
  EXPECT_NEAR(170000.0, crosswalk_info.polygon().points()[0].y(), 1E-3);
  EXPECT_NEAR(170003.0, crosswalk_info.polygon().points()[1].x(), 1E-3);
  EXPECT_NEAR(170000.0, crosswalk_info.polygon().points()[1].y(), 1E-3);
  EXPECT_NEAR(170003.0, crosswalk_info.polygon().points()[2].x(), 1E-3);
  EXPECT_NEAR(170003.0, crosswalk_info.polygon().points()[2].y(), 1E-3);
  EXPECT_NEAR(170000.0, crosswalk_info.polygon().points()[3].x(), 1E-3);
  EXPECT_NEAR(170003.0, crosswalk_info.polygon().points()[3].y(), 1E-3);
}

TEST_F(HDMapCommonTestSuite, TestStopSignInfo) {
  StopSign stop_sign;
  InitStopSignObj(&stop_sign);
  StopSignInfo stop_sign_info(stop_sign);
  EXPECT_EQ(stop_sign.id().id(), stop_sign_info.id().id());
  EXPECT_EQ(stop_sign.stop_line(0).segment(0).line_segment().point_size() - 1,
            stop_sign_info.segments().size());
  for (const auto &segment : stop_sign_info.segments()) {
    EXPECT_NEAR(1.0, segment.length(), 1E-4);
  }
}

TEST_F(HDMapCommonTestSuite, TestYieldSignInfo) {
  YieldSign yield_sign;
  InitYieldSignObj(&yield_sign);
  YieldSignInfo yield_sign_info(yield_sign);
  EXPECT_EQ(yield_sign.id().id(), yield_sign_info.id().id());
  EXPECT_EQ(yield_sign.stop_line(0).segment(0).line_segment().point_size() - 1,
            yield_sign_info.segments().size());
  for (const auto &segment : yield_sign_info.segments()) {
    EXPECT_NEAR(1.0, segment.length(), 1E-4);
  }
}

TEST_F(HDMapCommonTestSuite, TestClearAreaInfo) {
  ClearArea clear_area;
  InitClearAreaObj(&clear_area);
  ClearAreaInfo clear_area_info(clear_area);
  EXPECT_EQ(clear_area.id().id(), clear_area_info.id().id());
  EXPECT_EQ(4, clear_area_info.clear_area().polygon().point_size());
  EXPECT_NEAR(170000.0, clear_area_info.polygon().points()[0].x(), 1E-3);
  EXPECT_NEAR(170000.0, clear_area_info.polygon().points()[0].y(), 1E-3);
  EXPECT_NEAR(170003.0, clear_area_info.polygon().points()[1].x(), 1E-3);
  EXPECT_NEAR(170000.0, clear_area_info.polygon().points()[1].y(), 1E-3);
  EXPECT_NEAR(170003.0, clear_area_info.polygon().points()[2].x(), 1E-3);
  EXPECT_NEAR(170003.0, clear_area_info.polygon().points()[2].y(), 1E-3);
  EXPECT_NEAR(170000.0, clear_area_info.polygon().points()[3].x(), 1E-3);
  EXPECT_NEAR(170003.0, clear_area_info.polygon().points()[3].y(), 1E-3);
}

TEST_F(HDMapCommonTestSuite, TestSpeedBumpInfo) {
  SpeedBump speed_bump;
  InitSpeedBumpObj(&speed_bump);
  SpeedBumpInfo speed_bump_info(speed_bump);
  EXPECT_EQ(speed_bump.id().id(), speed_bump_info.id().id());
  EXPECT_EQ(speed_bump.position(0).segment(0).line_segment().point_size() - 1,
            speed_bump_info.segments().size());
  EXPECT_NEAR(1.0, speed_bump.speed(), 1E-4);
  for (const auto &segment : speed_bump_info.segments()) {
    EXPECT_NEAR(1.0, segment.length(), 1E-4);
  }
}

TEST_F(HDMapCommonTestSuite, TestRoadInfo) {
  Road road;
  InitRoadObj(&road);
  RoadInfo road_info(road);
  EXPECT_EQ(road.id().id(), road_info.id().id());
  EXPECT_EQ(2, road_info.sections().size());

  const RoadSection &section0 = road_info.sections()[0];
  EXPECT_EQ(section0.id().id(), "section_1");
  EXPECT_EQ(section0.lane_id_size(), 2);
  EXPECT_EQ(section0.lane_id(0).id(), "section_1_1");
  EXPECT_EQ(section0.lane_id(1).id(), "section_1_2");

  const RoadSection &section1 = road_info.sections()[1];
  EXPECT_EQ(section1.id().id(), "section_2");
  EXPECT_EQ(section1.lane_id_size(), 1);
  EXPECT_EQ(section1.lane_id(0).id(), "section_2_1");
}

TEST_F(HDMapCommonTestSuite, TestParkingSpaceInfo) {
  ParkingSpace parking_space;
  InitParkingSpaceObj(&parking_space);
  ParkingSpaceInfo parking_space_info(parking_space);
  EXPECT_EQ(parking_space.id().id(), parking_space_info.id().id());
  EXPECT_EQ(4, parking_space_info.parking_space().polygon().point_size());
  EXPECT_NEAR(170000.0, parking_space_info.polygon().points()[0].x(), 1E-3);
  EXPECT_NEAR(170000.0, parking_space_info.polygon().points()[0].y(), 1E-3);
  EXPECT_NEAR(170003.0, parking_space_info.polygon().points()[1].x(), 1E-3);
  EXPECT_NEAR(170000.0, parking_space_info.polygon().points()[1].y(), 1E-3);
  EXPECT_NEAR(170003.0, parking_space_info.polygon().points()[2].x(), 1E-3);
  EXPECT_NEAR(170003.0, parking_space_info.polygon().points()[2].y(), 1E-3);
  EXPECT_NEAR(170000.0, parking_space_info.polygon().points()[3].x(), 1E-3);
  EXPECT_NEAR(170003.0, parking_space_info.polygon().points()[3].y(), 1E-3);
}

TEST_F(HDMapCommonTestSuite, TestCleanAreaInfo) {
  CleanArea clean_area;
  InitCleanAreaObj(&clean_area);
  CleanAreaInfo clean_area_info(clean_area);
  EXPECT_EQ(clean_area_info.id().id(), clean_area.id().id());
  EXPECT_EQ(4, clean_area_info.cleanArea().polygon().point_size());
  EXPECT_NEAR(170000.0, clean_area_info.polygon().points()[0].x(), 1E-3);
  EXPECT_NEAR(170000.0, clean_area_info.polygon().points()[0].y(), 1E-3);
  EXPECT_NEAR(170003.0, clean_area_info.polygon().points()[1].x(), 1E-3);
  EXPECT_NEAR(170000.0, clean_area_info.polygon().points()[1].y(), 1E-3);
  EXPECT_NEAR(170003.0, clean_area_info.polygon().points()[2].x(), 1E-3);
  EXPECT_NEAR(170003.0, clean_area_info.polygon().points()[2].y(), 1E-3);
  EXPECT_NEAR(170000.0, clean_area_info.polygon().points()[3].x(), 1E-3);
  EXPECT_NEAR(170003.0, clean_area_info.polygon().points()[3].y(), 1E-3);
  EXPECT_EQ(true, clean_area_info.inCleanArea(Vec2d(170001.0, 170001.0)));
  EXPECT_EQ(false, clean_area_info.inCleanArea(Vec2d(170005.0, 170003.0)));
}

TEST_F(HDMapCommonTestSuite, TestSlopeAreaInfo) {
  SlopeArea slope_area;
  InitSlopeAreaObj(&slope_area);
  SlopeAreaInfo slope_area_info(slope_area);
  EXPECT_EQ(slope_area_info.id().id(), slope_area.id().id());
  EXPECT_EQ(4, slope_area_info.slopeArea().polygon().point_size());
  EXPECT_NEAR(170000.0, slope_area_info.polygon().points()[0].x(), 1E-3);
  EXPECT_NEAR(170000.0, slope_area_info.polygon().points()[0].y(), 1E-3);
  EXPECT_NEAR(170003.0, slope_area_info.polygon().points()[1].x(), 1E-3);
  EXPECT_NEAR(170000.0, slope_area_info.polygon().points()[1].y(), 1E-3);
  EXPECT_NEAR(170003.0, slope_area_info.polygon().points()[2].x(), 1E-3);
  EXPECT_NEAR(170003.0, slope_area_info.polygon().points()[2].y(), 1E-3);
  EXPECT_NEAR(170000.0, slope_area_info.polygon().points()[3].x(), 1E-3);
  EXPECT_NEAR(170003.0, slope_area_info.polygon().points()[3].y(), 1E-3);
  EXPECT_EQ(true, slope_area_info.inSlopeArea(Vec2d(170001.0, 170001.0)));
  EXPECT_EQ(false, slope_area_info.inSlopeArea(Vec2d(170005.0, 170003.0)));
  EXPECT_EQ(true, slope_area_info.hasAngle());
  EXPECT_EQ(true, slope_area_info.hasLength());
  EXPECT_EQ(true, slope_area_info.hasSpeed());
  EXPECT_NEAR(0.08, slope_area_info.angle(), 1E-5);
  EXPECT_NEAR(2.0, slope_area_info.length(), 1E-5);
  EXPECT_NEAR(0.2, slope_area_info.speed(), 1E-5);
}

TEST_F(HDMapCommonTestSuite, TestElevatorAreaInfo) {
  Elevator elevator;
  InitElevatorObj(&elevator);
  ElevatorInfo elevator_info(elevator);
  EXPECT_EQ(elevator_info.id().id(), elevator.id().id());
  EXPECT_EQ(4, elevator_info.elevator().polygon().point_size());
  EXPECT_NEAR(170000.0, elevator_info.polygon().points()[0].x(), 1E-3);
  EXPECT_NEAR(170000.0, elevator_info.polygon().points()[0].y(), 1E-3);
  EXPECT_NEAR(170003.0, elevator_info.polygon().points()[1].x(), 1E-3);
  EXPECT_NEAR(170000.0, elevator_info.polygon().points()[1].y(), 1E-3);
  EXPECT_NEAR(170003.0, elevator_info.polygon().points()[2].x(), 1E-3);
  EXPECT_NEAR(170003.0, elevator_info.polygon().points()[2].y(), 1E-3);
  EXPECT_NEAR(170000.0, elevator_info.polygon().points()[3].x(), 1E-3);
  EXPECT_NEAR(170003.0, elevator_info.polygon().points()[3].y(), 1E-3);
  EXPECT_EQ(true, elevator_info.inElevatorArea(Vec2d(170001.0, 170001.0)));
  EXPECT_EQ(false, elevator_info.inElevatorArea(Vec2d(170005.0, 170003.0)));
}

TEST_F(HDMapCommonTestSuite, TestNarrowAreaInfo) {
  NarrowArea narrow_area;
  InitNarrowAreaObj(&narrow_area);
  NarrowAreaInfo narrow_area_info(narrow_area);
  EXPECT_EQ(narrow_area_info.id().id(), narrow_area.id().id());
  EXPECT_EQ(4, narrow_area_info.narrowArea().polygon().point_size());
  EXPECT_NEAR(170000.0, narrow_area_info.polygon().points()[0].x(), 1E-3);
  EXPECT_NEAR(170000.0, narrow_area_info.polygon().points()[0].y(), 1E-3);
  EXPECT_NEAR(170003.0, narrow_area_info.polygon().points()[1].x(), 1E-3);
  EXPECT_NEAR(170000.0, narrow_area_info.polygon().points()[1].y(), 1E-3);
  EXPECT_NEAR(170003.0, narrow_area_info.polygon().points()[2].x(), 1E-3);
  EXPECT_NEAR(170003.0, narrow_area_info.polygon().points()[2].y(), 1E-3);
  EXPECT_NEAR(170000.0, narrow_area_info.polygon().points()[3].x(), 1E-3);
  EXPECT_NEAR(170003.0, narrow_area_info.polygon().points()[3].y(), 1E-3);
  EXPECT_EQ(true, narrow_area_info.inNarrowArea(Vec2d(170001.0, 170001.0)));
  EXPECT_EQ(false, narrow_area_info.inNarrowArea(Vec2d(170005.0, 170003.0)));
}

TEST_F(HDMapCommonTestSuite, TestMarkAreaInfo) {
  MarkArea mark_area;
  InitMarkAreaObj(&mark_area);
  MarkAreaInfo mark_area_info(mark_area);
  EXPECT_EQ(mark_area_info.id().id(), mark_area.id().id());
  EXPECT_EQ(4, mark_area_info.markArea().polygon().point_size());
  EXPECT_NEAR(170000.0, mark_area_info.polygon().points()[0].x(), 1E-3);
  EXPECT_NEAR(170000.0, mark_area_info.polygon().points()[0].y(), 1E-3);
  EXPECT_NEAR(170003.0, mark_area_info.polygon().points()[1].x(), 1E-3);
  EXPECT_NEAR(170000.0, mark_area_info.polygon().points()[1].y(), 1E-3);
  EXPECT_NEAR(170003.0, mark_area_info.polygon().points()[2].x(), 1E-3);
  EXPECT_NEAR(170003.0, mark_area_info.polygon().points()[2].y(), 1E-3);
  EXPECT_NEAR(170000.0, mark_area_info.polygon().points()[3].x(), 1E-3);
  EXPECT_NEAR(170003.0, mark_area_info.polygon().points()[3].y(), 1E-3);
  EXPECT_EQ(true, mark_area_info.inMarkArea(Vec2d(170001.0, 170001.0)));
  EXPECT_EQ(false, mark_area_info.inMarkArea(Vec2d(170005.0, 170003.0)));
  EXPECT_EQ(true, mark_area_info.hasSpeed());
  EXPECT_NEAR(0.6, mark_area_info.speed(), 1E-5);
  EXPECT_EQ(true, mark_area_info.hasMaterial());
  EXPECT_EQ("mark", mark_area.material());
}

TEST_F(HDMapCommonTestSuite, TestProhibitedAreaInfo) {
  ProhibitedArea prohibited_area;
  InitProhibitedObj(&prohibited_area);
  ProhibitedAreaInfo prohibited_area_info(prohibited_area);
  EXPECT_EQ(prohibited_area_info.id().id(), prohibited_area.id().id());
  EXPECT_EQ(4, prohibited_area_info.prohibitedArea().polygon().point_size());
  EXPECT_NEAR(170000.0, prohibited_area_info.polygon().points()[0].x(), 1E-3);
  EXPECT_NEAR(170000.0, prohibited_area_info.polygon().points()[0].y(), 1E-3);
  EXPECT_NEAR(170003.0, prohibited_area_info.polygon().points()[1].x(), 1E-3);
  EXPECT_NEAR(170000.0, prohibited_area_info.polygon().points()[1].y(), 1E-3);
  EXPECT_NEAR(170003.0, prohibited_area_info.polygon().points()[2].x(), 1E-3);
  EXPECT_NEAR(170003.0, prohibited_area_info.polygon().points()[2].y(), 1E-3);
  EXPECT_NEAR(170000.0, prohibited_area_info.polygon().points()[3].x(), 1E-3);
  EXPECT_NEAR(170003.0, prohibited_area_info.polygon().points()[3].y(), 1E-3);
  EXPECT_EQ(true,
            prohibited_area_info.inProhibitedArea(Vec2d(170001.0, 170001.0)));
  EXPECT_EQ(false,
            prohibited_area_info.inProhibitedArea(Vec2d(170005.0, 170003.0)));
}

}  // namespace hdmap
}  // namespace cvte

int main(int argc, char **argv) {
  testing::InitGoogleTest();
  return RUN_ALL_TESTS();
}