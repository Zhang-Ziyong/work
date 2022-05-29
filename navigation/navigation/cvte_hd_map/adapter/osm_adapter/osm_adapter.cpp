#include "osm_adapter.hpp"
#include "OsmOperator.hpp"
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>
#include <boost/filesystem.hpp>
#include <tinyxml2.h>
#include "common_define.hpp"
#include "glog/logging.h"

namespace cvte {
namespace hdmap {

Id CreateHDMapId(const std::string &string_id) {
  Id id;
  id.set_id(string_id);
  return id;
}

OsmAdapter::OsmAdapter() {}

bool OsmAdapter::LoadData(const std::string &filename, Map *pb_map) {
  if (pb_map == nullptr) {
    LOG(ERROR) << "pb_map is nullptr";
    return false;
  }
  pb_map->Clear();
  CVTE_BABOT::OsmOperator osm_operator;
  CVTE_BABOT::MapDetail map_detail;
  osm_operator.getMapDetail(filename, map_detail);
  if (map_detail.vec_region.empty()) {
    LOG(ERROR) << "osm region is empty";
    return false;
  }
  MapArea map_area;
  map_area.mutable_id()->set_id(filename);
  for (const auto &it : map_detail.vec_region) {
    if (it.region_type == "ridge") {
      LOG(INFO) << "read pit(ridge) area " << it.region_id;
      PbPitArea pit_area;
      map_area.add_cleanarea_id()->set_id(it.region_id);
      setAreaAttribute(it, pit_area);
      *(pb_map->add_pit_area()) = pit_area;
    } else if (it.region_type == "slope") {
      LOG(INFO) << "read slope area " << it.region_id;
      PbSlopeArea slope_area;
      map_area.add_cleanarea_id()->set_id(it.region_id);
      setAreaAttribute(it, slope_area);
      *(pb_map->add_slope_area()) = slope_area;
    } else if (it.region_type == "narrow_aisle") {
      LOG(INFO) << "read narrow_aisle " << it.region_id;
      PbNarrowArea narrow_area;
      map_area.add_cleanarea_id()->set_id(it.region_id);
      setAreaAttribute(it, narrow_area);
      *(pb_map->add_narrow_area()) = narrow_area;
    } else if (it.region_type == "speed_hump") {
      LOG(INFO) << "read speed_hump " << it.region_id;
      PbSpeedBump speed_bump;
      map_area.add_cleanarea_id()->set_id(it.region_id);
      setAreaAttribute(it, speed_bump);
      *(pb_map->add_speed_bump()) = speed_bump;
    } else if (it.region_type == "business") {
      LOG(INFO) << "read business " << it.region_id;
      PbCleanArea clean_area;
      map_area.add_cleanarea_id()->set_id(it.region_id);
      setAreaAttribute(it, clean_area);
      *(pb_map->add_clean_area()) = clean_area;
    } else if (it.region_type == "mark_area") {
      LOG(INFO) << "read mark_area " << it.region_id;
      PbMarkArea mark_area;
      map_area.add_cleanarea_id()->set_id(it.region_id);
      setAreaAttribute(it, mark_area);
      *(pb_map->add_mark_area()) = mark_area;
    } else if (it.region_type == "elevator") {
      LOG(INFO) << "read elevator area " << it.region_id;
      PbElevator elevator_area;
      map_area.add_cleanarea_id()->set_id(it.region_id);
      elevator_area.mutable_id()->set_id(it.region_id);
      PbPolygon *polygon = elevator_area.mutable_polygon();
      for (const auto &point : it.vec_coordinate) {
        LOG(INFO) << "point: " << point.x << " " << point.y;
        Point2d *pbpoint = polygon->add_point();
        pbpoint->set_x(point.x);
        pbpoint->set_y(point.y);
      }
      *(pb_map->add_elevator()) = elevator_area;
    } else {
      LOG(ERROR) << "no such region: " << it.region_type;
    }
  }
  // for (const auto &it : map_detail.vec_point) {
  //   if (it.point_type == "take_lift") {
  //     LOG(INFO) << "read take_lift point";
  //     CVTE_BABOT::Coordinate point1 = it.coordinate;
  //     if (it.extend_attributes.find("cvte_access_lift_point_id") !=
  //         it.extend_attributes.end()) {
  //       std::string access_point_id =
  //           it.extend_attributes.find("cvte_access_lift_point_id")->second;
  //       // for (const auto &p_it : map_detail.vec_point) {
  //       for (size_t index = 0; index < map_detail.vec_point.size(); index++)
  //       {
  //         auto access_point = map_detail.vec_point[index];
  //         if (access_point.coordinate.point_id == access_point_id) {
  //           CVTE_BABOT::Coordinate point2 = access_point.coordinate;
  //           PbElevator elevator_area;
  //           elevator_area.mutable_id()->set_id(it.coordinate.point_id);
  //           std::vector<math_utils::Vec2d> pts =
  //               generaRect(math_utils::Vec2d(point1.x, point1.y),
  //                          math_utils::Vec2d(point2.x, point2.y));
  //           LOG(INFO) << "construct elevator polygon finish";
  //           PbPolygon *polygon = elevator_area.mutable_polygon();
  //           for (const auto &pt : pts) {
  //             LOG(INFO) << "point: " << pt(0) << " " << pt(1);
  //             Point2d *pbpoint = polygon->add_point();
  //             pbpoint->set_x(pt(0));
  //             pbpoint->set_y(pt(1));
  //           }
  //           map_area.add_cleanarea_id()->set_id(it.coordinate.point_id);
  //           *(pb_map->add_elevator()) = elevator_area;
  //         }
  //       }
  //     } else {
  //       LOG(ERROR) << "can not find access lift point";
  //     }
  //   }
  // }
  *(pb_map->add_map_area()) = map_area;
  return true;
}
template <class AttributeT, class AreaT>
void OsmAdapter::setAreaAttribute(const AttributeT &attribute, AreaT &region) {
  region.mutable_id()->set_id(attribute.region_id);
  PbPolygon *polygon = region.mutable_polygon();
  for (const auto &point : attribute.vec_coordinate) {
    LOG(INFO) << "point: " << point.x << " " << point.y;
    Point2d *pbpoint = polygon->add_point();
    pbpoint->set_x(point.x);
    pbpoint->set_y(point.y);
  }
  if (attribute.extend_attributes.find("cvte_speed_limit") !=
      attribute.extend_attributes.end()) {
    const auto speed = attribute.extend_attributes.find("cvte_speed_limit");
    region.set_speed(std::stod(speed->second));
  }
  if (attribute.extend_attributes.find("cvte_floor_material") !=
      attribute.extend_attributes.end()) {
    const auto material =
        attribute.extend_attributes.find("cvte_floor_material");
    region.set_material(material->second);
    LOG(INFO) << "floor material: " << material->second;
  }
  if (attribute.extend_attributes.find("cvte_floor_color") !=
      attribute.extend_attributes.end()) {
    const auto color = attribute.extend_attributes.find("cvte_floor_color");
    region.set_color(color->second);
    LOG(INFO) << "floor_color : " << color->second;
  }
  if (attribute.extend_attributes.find("cvte_is_static") !=
      attribute.extend_attributes.end()) {
    const auto is_static = attribute.extend_attributes.find("cvte_is_static");
    region.set_static_area(is_static->second);
  }
}

std::vector<math_utils::Vec2d> OsmAdapter::generaRect(
    const math_utils::Vec2d &point1, const math_utils::Vec2d &point2) {
  math_utils::Vec2d line1 = (point1 - point2).normalized();
  math_utils::Vec2d line2(line1(1), -line1(0));
  math_utils::Vec2d middle = (point2 + point1) / 2;
  std::vector<math_utils::Vec2d> points;
  double length = 1.2;
  double add_length = 0.4;
  math_utils::Vec2d pt = point1 + length * line2 + add_length * line1;
  points.push_back(pt);
  pt = point1 - length * line2 + add_length * line1;
  points.push_back(pt);
  pt = point2 + length * line2 - add_length * line1;
  points.push_back(pt);
  pt = point2 - length * line2 - add_length * line1;
  points.push_back(pt);
  // math_utils::Vec2d pt = middle + length * line1 + length * line2;
  // points.push_back(pt);
  // pt = middle + length * line1 - length * line2;
  // points.push_back(pt);
  // pt = middle - length * line1 - length * line2;
  // points.push_back(pt);
  // pt = middle - length * line1 + length * line2;
  // points.push_back(pt);
  return points;
}

}  // namespace hdmap
}  // namespace cvte