/*
 * polygon_coverage_planning implements algorithms for coverage planning in
 * general polygons with holes. Copyright (C) 2019, Rik Bähnemann, Autonomous
 * Systems Lab, ETH Zürich
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ros_interface.h"
#include <rclcpp/rclcpp.hpp>

#include <assert.h>
#include <polygon_coverage_geometry/boolean.h>
#include <polygon_coverage_geometry/cgal_comm.h>
#include <polygon_coverage_geometry/cgal_definitions.h>
#include <polygon_coverage_geometry/triangulation.h>

#include <algorithm>
#include <limits>
#include "user_glog.hpp"

void poseArrayMsgFromPath(
    const std::vector<Point_2> &waypoints, double altitude,
    const std::string &frame_id,
    geometry_msgs::msg::PoseArray *trajectory_points_pose_array) {
  assert(trajectory_points_pose_array);
  if (waypoints.empty()) {
    return;
  }

  trajectory_points_pose_array->poses.reserve(waypoints.size());
  // Header
  trajectory_points_pose_array->header.frame_id = frame_id;
  for (size_t i = 0; i < waypoints.size(); i++) {
    geometry_msgs::msg::Pose msg;
    msg.position.x = CGAL::to_double(waypoints[i].x());
    msg.position.y = CGAL::to_double(waypoints[i].y());
    msg.position.z = altitude;
    trajectory_points_pose_array->poses.push_back(msg);
  }
}

void createMarkers(const std::vector<Point_2> &vertices, double altitude,
                   const std::string &frame_id, const std::string &ns,
                   const Color &points_color, const Color &lines_color,
                   const double line_size, const double point_size,
                   visualization_msgs::msg::Marker *points,
                   visualization_msgs::msg::Marker *line_strip) {
  assert(points);
  assert(line_strip);
  points->points.clear();
  line_strip->points.clear();
  auto ptr_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  points->header.frame_id = line_strip->header.frame_id = frame_id;
  points->header.stamp = line_strip->header.stamp = ptr_clock->now();
  points->ns = line_strip->ns = ns;
  points->action = line_strip->action = visualization_msgs::msg::Marker::ADD;
  points->pose.orientation.w = line_strip->pose.orientation.w = 1.0;

  points->id = 0;
  line_strip->id = 1;

  points->type = visualization_msgs::msg::Marker::POINTS;
  line_strip->type = visualization_msgs::msg::Marker::LINE_STRIP;

  points->scale.x = point_size;
  points->scale.y = point_size;
  line_strip->scale.x = line_size;

  points->color = points_color;
  line_strip->color = lines_color;

  for (size_t i = 0; i < vertices.size(); i++) {
    geometry_msgs::msg::Point p;
    p.x = CGAL::to_double(vertices[i].x());
    p.y = CGAL::to_double(vertices[i].y());
    p.z = altitude;

    points->points.push_back(p);
    line_strip->points.push_back(p);
  }
}

void createPolygonMarkers(const PolygonWithHoles &polygon, double altitude,
                          const std::string &frame_id, const std::string &ns,
                          const Color &polygon_color, const Color &hole_color,
                          const double line_size, const double point_size,
                          visualization_msgs::msg::MarkerArray *array) {
  assert(array);
  array->markers.clear();

  // Polygon markers.
  visualization_msgs::msg::Marker hull_points, hull_vertices;
  // Hull.
  std::vector<Point_2> hull = getHullVertices(polygon);
  hull.push_back(hull.front());
  createMarkers(hull, altitude, frame_id, ns + "hull", polygon_color,
                polygon_color, line_size, point_size, &hull_points,
                &hull_vertices);
  array->markers.push_back(hull_points);
  array->markers.push_back(hull_vertices);

  // Hole markers:
  size_t i = 0;
  for (auto h = polygon.holes_begin(); h != polygon.holes_end(); ++h) {
    visualization_msgs::msg::Marker hole_tris;
    // Faces.
    std::vector<std::vector<Point_2>> triangles;
    triangulatePolygon(PolygonWithHoles(*h), &triangles);
    createTriangles(triangles, frame_id,
                    ns + "hole_mesh_" + std::to_string(i++), hole_color,
                    altitude, &hole_tris);
    array->markers.push_back(hole_tris);
  }
}

void createStartAndEndPointMarkers(const Point_2 &start, const Point_2 &end,
                                   double altitude, const std::string &frame_id,
                                   const std::string &ns,
                                   visualization_msgs::msg::Marker *start_point,
                                   visualization_msgs::msg::Marker *end_point) {
  assert(start_point);
  assert(end_point);
  auto ptr_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  start_point->header.frame_id = end_point->header.frame_id = frame_id;
  start_point->header.stamp = end_point->header.stamp = ptr_clock->now();
  start_point->ns = ns + "_start";
  end_point->ns = ns + "_end";
  start_point->action = end_point->action =
      visualization_msgs::msg::Marker::ADD;

  geometry_msgs::msg::Pose start_msg, end_msg;

  start_msg.position.x = CGAL::to_double(start.x());
  start_msg.position.y = CGAL::to_double(start.y());
  start_msg.position.z = altitude;

  end_msg.position.x = CGAL::to_double(end.x());
  end_msg.position.y = CGAL::to_double(end.y());
  end_msg.position.z = altitude;

  start_point->pose = start_msg;
  end_point->pose = end_msg;

  start_point->pose.orientation.w = end_point->pose.orientation.w = 1.0;

  start_point->id = end_point->id = 0;

  start_point->type = end_point->type = visualization_msgs::msg::Marker::SPHERE;

  start_point->scale.x = end_point->scale.x = 1.0;
  start_point->scale.y = end_point->scale.y = 1.0;
  start_point->scale.z = end_point->scale.z = 1.0;

  start_point->color = Color::Green();
  start_point->color.a = 0.5;

  end_point->color = Color::Red();
  end_point->color.a = 0.5;
}

void createStartAndEndTextMarkers(const Point_2 &start, const Point_2 &end,
                                  double altitude, const std::string &frame_id,
                                  const std::string &ns,
                                  visualization_msgs::msg::Marker *start_text,
                                  visualization_msgs::msg::Marker *end_text) {
  assert(start_text);
  assert(end_text);
  auto ptr_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  start_text->header.frame_id = end_text->header.frame_id = frame_id;
  start_text->header.stamp = end_text->header.stamp = ptr_clock->now();
  start_text->ns = ns + "_start_text";
  start_text->ns = ns + "_end_text";
  start_text->action = end_text->action = visualization_msgs::msg::Marker::ADD;
  start_text->type = end_text->type =
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

  geometry_msgs::msg::Pose start_msg, end_msg;

  start_msg.position.x = CGAL::to_double(start.x());
  start_msg.position.y = CGAL::to_double(start.y());
  start_msg.position.z = altitude;

  end_msg.position.x = CGAL::to_double(end.x());
  end_msg.position.y = CGAL::to_double(end.y());
  end_msg.position.z = altitude;

  start_text->pose = start_msg;
  end_text->pose = end_msg;

  start_text->pose.orientation.w = end_text->pose.orientation.w = 1.0;

  start_text->text = "S";
  start_text->color = Color::Black();

  end_text->text = "G";
  end_text->color = Color::Black();

  start_text->scale.z = end_text->scale.z = 1.0;
}

void polygon2FromPolygonMsg(const geometry_msgs::msg::Polygon &msg,
                            Polygon_2 *polygon) {
  assert(polygon);

  std::vector<Point_2> vertices(msg.points.size());
  for (size_t i = 0; i < msg.points.size(); ++i)
    vertices[i] = Point_2(msg.points[i].x, msg.points[i].y);

  *polygon = Polygon_2(vertices.begin(), vertices.end());
}

bool polygonFromMsg(
    const polygon_coverage_msgs::msg::PolygonWithHolesStamped &msg,
    PolygonWithHoles *polygon, double *altitude, std::string *frame) {
  assert(polygon);
  assert(altitude);
  assert(frame);

  *frame = msg.header.frame_id;

  if (msg.polygon.hull.points.size() > 0) {
    *altitude = msg.polygon.hull.points[0].z;
    INFO_STREAM(
        "Setting polygon altitude height to first z variable: " << *altitude);
  } else {
    ERROR_STREAM("Polygon hull data empty. Cannot set altitude.");
    return false;
  }

  Polygon_2 hull;
  polygon2FromPolygonMsg(msg.polygon.hull, &hull);
  if (hull.is_clockwise_oriented())
    hull.reverse_orientation();

  std::list<Polygon_2> holes(msg.polygon.holes.size());
  for (size_t i = 0; i < msg.polygon.holes.size(); ++i) {
    auto hole = std::next(holes.begin(), i);
    polygon2FromPolygonMsg(msg.polygon.holes[i], &(*hole));
    if (hole->is_counterclockwise_oriented())
      hole->reverse_orientation();
  }

  // Cut holes from hull.
  std::list<PolygonWithHoles> res =
      computeDifference(hull, holes.begin(), holes.end());
  if (res.empty()) {
    ERROR_STREAM("Failed to create polygon with holes from msg.");
    return false;
  }
  *polygon = res.front();

  if (polygon->outer_boundary().size() < 3) {
    ERROR_STREAM("Input polygon is not valid.");
    return false;
  } else if (!isStrictlySimple(*polygon)) {
    ERROR_STREAM("Input polygon is not simple.");
    return false;
  } else if (res.size() > 1) {
    ERROR_STREAM(
        "Failed to create polygon from message. More than two input polygons "
        "after cropping holes from hull.");
    return false;
  }

  return true;
}
void createTriangles(const std::vector<std::vector<Point_2>> &triangles,
                     const std::string &frame_id, const std::string &ns,
                     const Color &color, const double altitude,
                     visualization_msgs::msg::Marker *markers) {
  assert(markers);

  markers->points.clear();
  auto ptr_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  markers->header.frame_id = frame_id;
  markers->header.stamp = ptr_clock->now();
  markers->ns = ns;
  markers->action = visualization_msgs::msg::Marker::ADD;
  markers->pose.orientation.w = 1.0;
  markers->scale.x = 1.0;
  markers->scale.y = 1.0;
  markers->scale.z = 1.0;

  markers->id = 0;
  markers->type = visualization_msgs::msg::Marker::TRIANGLE_LIST;

  markers->color = color;
  for (const auto &t : triangles) {
    assert(t.size() == 3);
    for (const auto &v : t) {
      geometry_msgs::msg::Point p;
      p.x = CGAL::to_double(v.x());
      p.y = CGAL::to_double(v.y());
      p.z = altitude;

      markers->points.push_back(p);
    }
  }
}
