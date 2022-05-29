/*
 * @Author: chennuo@cvte.com 
 * @Date: 2021-06-22 17:04:52 
 * @Last Modified by: chennuo@cvte.com
 * @Last Modified time: 2021-06-22 18:21:13
 * @brief 原理: 从检测点引出一条射线，统计该射线与多边形的交点个数，如果个数是奇数则该点在多边形内
 */

#pragma once

#include <vector>
#include <limits>
#include <algorithm>
#include <math.h>

namespace CVTE_BABOT
{

template <typename T>
struct Point2d {
    T x;
    T y;

    Point2d(T x, T y) {
        this->x = x;
        this->y = y;
    }

    Point2d(const Point2d& p) {
        this->x = p.x;
        this->y = p.y;
    }

    Point2d& operator = (const Point2d& p) {
        this->x = p.x;
        this->y = p.y;
        return *this;
    }

};

struct BoundingBox {
    int xmin;
    int xmax;
    int ymin;
    int ymax;
    
    BoundingBox(int xmin, int xmax, int ymin, int ymax) {
        this->xmin = xmin;
        this->xmax = xmax;
        this->ymin = ymin;
        this->ymax = ymax;
    }

    BoundingBox() {
        this->xmin = std::numeric_limits<int>::max();
        this->xmax = std::numeric_limits<int>::min();
        this->ymin = std::numeric_limits<int>::max();
        this->ymax = std::numeric_limits<int>::min();
    }
};

struct Polygon {
    std::vector<Point2d<int>> points;
    BoundingBox bbox;

    Polygon(std::vector<Point2d<int>>& points): points(points) {

        // 判断多边形是否闭环，如果未闭环，则将起点作为多边形最后一个点，保证闭环
        if (points.size() >= 3) {
            if ((points.front().x != points.back().x) || (points.front().y != points.back().y)){
                points.push_back(points.front());
            }
        }

        calcBoundingBox(points);
    }

    int direction(Point2d<int> pi, Point2d<int> pj, Point2d<int> pk) {
        return (pk.x - pi.x) * (pj.y - pi.y) - (pj.x - pi.x) * (pk.y - pi.y);
    }

    bool onSegment(Point2d<int> pi, Point2d<int> pj, Point2d<int> pk) {
        if (std::min(pi.x, pj.x) <= pk.x && pk.x <= std::max(pi.x, pj.x)
            && std::min(pi.y, pj.y) <= pk.y && pk.y <= std::max(pi.y, pj.y)) {
            return true;
        } else {
            return false;
        }
    }

    bool segmentIntersect(Point2d<int> p1, Point2d<int> p2, Point2d<int> p3, Point2d<int> p4) {
        int d1 = direction(p3, p4, p1);
        int d2 = direction(p3, p4, p2);
        int d3 = direction(p1, p2, p3);
        int d4 = direction(p1, p2, p4);

        if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
            return true;
        } else if (d1 == 0 && onSegment(p3, p4, p1)) {
            return true;
        } else if (d2 == 0 && onSegment(p3, p4, p2)) {
            return true;
        } else if (d3 == 0 && onSegment(p1, p2, p3)) {
            return true;
        } else if (d4 == 0 && onSegment(p1, p2, p4)) {
            return true;
        } else {
            return false;
        }
    }

    bool inBoundingBox(Point2d<int> point) {
        if (point.x < bbox.xmin || point.x > bbox.xmax || point.y < bbox.ymin || point.y > bbox.ymax) {
            return false;
        } else {
            return true;
        }
    }

    bool inPolygon(Point2d<int> point) {
        if (!inBoundingBox(point)) {
            return false;
        }

        Point2d<int> outside(bbox.xmin - 1, bbox.ymin);
        int intersections = 0;
        // 检查从当前点出发的射线与多边形边的交点数量
        for (int i = 0; i < points.size() - 1; ++i) {
            if (segmentIntersect(point, outside, points.at(i), points.at(i + 1))) {
                intersections++;
            }
        }
        if (segmentIntersect(point, outside, points.at(points.size() - 1), points.at(0))) {
            intersections++;
        }
        return (intersections % 2 != 0);
    }
    
    // 判断点是否在多边形边界上
    bool pointOnBorder(Point2d<int> point) {
        if (points.size() < 3) {
            return false;
        }

        for (int i = 0; i < points.size() - 1; i++) {
            // 直线方程定义: ax + by + c = (y2 - y1)x + (x1 - x2)y + (x2y1 - x1y2) = 0;
            // 点到直线的距离: |ax + by + c| / sqrt(a*a + b*b)
            const auto& p1 = points[i];
            const auto& p2 = points[i + 1];
            double a = p2.y - p1.y;
            double b = p1.x - p2.x;
            double c = p2.x * p1.y - p1.x * p2.y;
            if (0 == a && 0 == b) {
                return true;
            }
            
            // 判断点到直线的距离是否超过一个栅格大小，如果超过说明当前点不在直线上
            double dis = fabs(a * point.x + b * point.y + c) / sqrt(a * a + b * b);
            if (dis < 1.0) {
                return true;
            }
        }

        return false;
    }
    
    // 判断点是否在多边形内(不包含边界)
    bool inPolygonWithoutBorder(Point2d<int> point) {
        if (!inPolygon(point) || points.size() < 3) {
            return false;
        }
        
        // 检查当前点是否在边界上
        if (pointOnBorder(point)) {
            return false;
        }

        return true;
    }

private:

    void calcBoundingBox(std::vector<Point2d<int>> points) {
        for (auto &point : points) {
            if (point.x < bbox.xmin) {
                bbox.xmin = point.x;
            } else if (point.x > bbox.xmax) {
                bbox.xmax = point.x;
            }
            if (point.y < bbox.ymin) {
                bbox.ymin = point.y;
            } else if (point.y > bbox.ymax) {
                bbox.ymax = point.y;
            }
        }
    }
};

}