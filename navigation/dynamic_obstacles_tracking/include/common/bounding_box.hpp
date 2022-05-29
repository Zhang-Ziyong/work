#ifndef COMMON_INCLUDE_COMMON_BOUNDING_BOX_HPP_
#define COMMON_INCLUDE_COMMON_BOUNDING_BOX_HPP_

#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <boost/numeric/ublas/matrix.hpp>

#include <boost/geometry/geometries/adapted/c_array.hpp>
BOOST_GEOMETRY_REGISTER_C_ARRAY_CS(cs::cartesian)

#include "tracking_utils.hpp"

namespace CVTE_BABOT {
/*
 *                           |x
 *(x_max,y_max) ---width-----|
 *      |                    |
 *      |                  length
 *      |                    |
 *  y<-----------------(x_min,y_min)
 */
typedef struct {
  double x_max;  // left-top corner
  double y_max;
  double x_min;  // right-bottom corner
  double y_min;
} BoundingBox;

// Orientation Bounding Box
typedef struct {
  double gc_x, gc_y, gc_z;
  double yaw_rad;
  double h, w, l;
} GroundBox;

typedef boost::geometry::model::polygon<
    boost::geometry::model::d2::point_xy<double>>
    Polygon;

/**
 * @brief Object's 3D OBB to 2D ground box
 * @param object
 * @param gbox
 */
static void toGroundBox(ConstObjectPtr object, GroundBox *gbox) {
  gbox->gc_x = object->object_center(0);
  gbox->gc_y = object->object_center(1);
  gbox->gc_z = 0;
  gbox->yaw_rad = 0.0;
  gbox->h = 0;
  gbox->w = object->width;
  gbox->l = object->length;
}

/*
 * @brief compute polygon of an oriented bounding box
 * @note Apollo's Object Coordinate
 *          |x
 *      C   |   D-----------
 *          |              |
 *  y---------------     length
 *          |              |
 *      B   |   A-----------
 */
template <typename T>
Polygon toPolygon(const T &g) {
  using boost::numeric::ublas::matrix;
  matrix<double> mref(2, 2);
  mref(0, 0) = cos(g.yaw_rad);
  mref(0, 1) = -sin(g.yaw_rad);
  mref(1, 0) = sin(g.yaw_rad);
  mref(1, 1) = cos(g.yaw_rad);

  matrix<double> corners(2, 4);
  // -------------(l/2,w/2)(l/2,-w/2)(-l/2,-w/2)(-l/2,w/2)
  double data[] = {g.l / 2, g.l / 2,  -g.l / 2, -g.l / 2,
                   g.w / 2, -g.w / 2, -g.w / 2, g.w / 2};
  std::copy(data, data + 8, corners.data().begin());
  matrix<double> gc = boost::numeric::ublas::prod(mref, corners);

  for (int i = 0; i < 4; ++i) {
    gc(0, i) += g.gc_x;
    gc(1, i) += g.gc_y;
  }

  double points[][2] = {{gc(0, 0), gc(1, 0)},
                        {gc(0, 1), gc(1, 1)},
                        {gc(0, 2), gc(1, 2)},
                        {gc(0, 3), gc(1, 3)},
                        {gc(0, 0), gc(1, 0)}};
  Polygon poly;
  boost::geometry::append(poly, points);
  return poly;
}

/**
 * @brief Intersection-over-Union
 */
static double groundBoxIoU(const GroundBox &box1, const GroundBox &box2) {
  Polygon gp = toPolygon(box1);
  Polygon dp = toPolygon(box2);

  std::vector<Polygon> in, un;
  boost::geometry::intersection(gp, dp, in);
  boost::geometry::union_(gp, dp, un);

  double inter_area = in.empty() ? 0. : boost::geometry::area(in.front());
  // double union_area = boost::geometry::area(un.front());

  // double o = 0.;
  // union
  // o = inter_area / union_area;
  // bbox_a
  // o = inter_area / area(dp);
  // bbox_b
  // o = inter_area / area(gp);
  // return o;

  double o1 = inter_area / boost::geometry::area(dp);
  double o2 = inter_area / boost::geometry::area(gp);
  return std::max(o1, o2);
}

static bool groundBoxInside(const GroundBox &box1, const GroundBox &box2) {
  Polygon gp = toPolygon(box1);
  Polygon dp = toPolygon(box2);

  std::vector<Polygon> in;
  boost::geometry::intersection(gp, dp, in);
  if (in.empty()) {
    return false;
  } else {
    double inter_area = boost::geometry::area(in.front());
    double box1_area = boost::geometry::area(gp);
    return abs(box1_area - inter_area) < EPSILON;
  }
}

/**
 * @brief check box1 is overlapping with box2
 *  true: box1 is inside box2 or box2 is inside box1
 *  true: IoU between box1 and box2 > threshold_IoU
 * @param box1
 * @param box2
 * @param threshold_IoU
 * @return
 */
static bool groundBoxOverlap(const GroundBox &box1, const GroundBox &box2,
                             double threshold_IoU) {
  if (groundBoxInside(box1, box2) || groundBoxInside(box2, box1)) {
    return true;
  }

  if (groundBoxIoU(box1, box2) > threshold_IoU) {
    return true;
  }

  return false;
}
}  // namespace CVTE_BABOT

#endif  // COMMON_INCLUDE_COMMON_BOUNDING_BOX_HPP_
