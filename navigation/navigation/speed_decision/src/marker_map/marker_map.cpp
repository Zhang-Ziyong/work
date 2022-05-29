/*
 * @Author: chennuo@cvte.com 
 * @Date: 2021-06-22 14:34:24 
 * @Last Modified by: chennuo@cvte.com
 * @Last Modified time: 2021-06-22 21:01:11
 */

#include <glog/logging.h>
#include <queue>
#include <time.h>
#include <unordered_set>
#include <map>

#include "marker_map.hpp"

namespace CVTE_BABOT
{

MarkerMap::MarkerMap()
{

}

MarkerMap::~MarkerMap()
{
  
}

void MarkerMap::resetMap(const size_t &size_x, const size_t &size_y, 
  const double &resolution, const double &origin_x, const double &origin_y)
{
  ptr_map_.reset(new Costmap2d(size_x, size_y, resolution, origin_x, origin_y, 0));
  
  has_set_map_ = true;
}

void MarkerMap::resetMap(const std::shared_ptr<Costmap2d>& costmap)
{
  ptr_map_.reset(new Costmap2d(costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), 
    costmap->getResolution(), costmap->getOriginX(), costmap->getOriginY(), 0));
  
  has_set_map_ = true;
}

void MarkerMap::bresenham(const int& sx, const int& sy, const int& tx, const int& ty, std::vector<std::pair<int, int> >& indexs)
{
  int x,y,dx,dy,unitx,unity,fabs_dx,fabs_dy,e;
  dx= tx-sx;
  dy= ty-sy;
  fabs_dx = fabs(dx);
  fabs_dy = fabs(dy);
  if (fabs_dx == 0) {
    unitx = 0;
  } else {
    unitx = dx / fabs_dx ;
  }
  if (fabs_dy == 0) {
    unity = 0;
  }
  else {
    unity = dy / fabs_dy ;
  }

  x= sx;
  y= sy;
  if( fabs_dx > fabs_dy ) {
    e=-fabs_dx;
    for(int i=0;i<=fabs_dx;i++) {
      std::pair<int, int> index(x, y);
      indexs.push_back(index);
      x+=unitx,e=e+2*fabs_dy;
      if(e>=0) {
        y+=unity;e=e-2*fabs_dx;
      }
    } 
  } else {
    e=-fabs_dy;
    for(int i=0; i<=fabs_dy; i++) {
      std::pair<int, int> index(x, y);
      indexs.push_back(index);
      y+=unity,e=e+2*fabs_dx;
      if(e>=0) {
          x+=unitx;e=e-2*fabs_dy;
      }
    } 
  }
}

void MarkerMap::inflateBorder(const std::vector<Point2d<size_t>>& edge_points, uint8_t value)
{
  // 判断是否已经进行了地图初始化
  if (!ptr_map_) {
    LOG(WARNING) << "marker map not initialize !";
    return;
  }

  // 建立优先队列，用于由近及远来访问膨胀区域
  std::unordered_set<size_t> seen_set; // 已膨胀过的区域的hash值集合，用于判断栅格是否被膨胀过
  std::map<double, std::vector<GridData> > inflation_grids;
  std::vector<GridData>& first_bin = inflation_grids[0.0];
  for (const auto& bp : edge_points) {
    size_t hash_value = bp.x + bp.y * ptr_map_->getSizeInCellsX();
    first_bin.push_back(std::move(GridData(hash_value, bp.x, bp.y, bp.x, bp.y)));
    seen_set.insert(hash_value);
  }

  // 建立索引距离查找表，用于通过索引差值确定栅格距离
  size_t grid_inflation_radius = inflation_radius_ / ptr_map_->getResolution();
  std::vector<std::vector<double>> cached_distance(grid_inflation_radius + 5, std::vector<double>(grid_inflation_radius + 5, 0.0));
  for (int i = 0; i < grid_inflation_radius + 5; i++) {
    for (int j = 0; j < grid_inflation_radius + 5; j++) {
      cached_distance[i][j] = sqrt(i * i + j * j) * ptr_map_->getResolution();
    }
  }

  // 判断并添加膨胀区域栅格函数
  auto addGrid = [&](const GridData& grid, size_t x, size_t y) {
    size_t hash_value = x + y * ptr_map_->getSizeInCellsX();
    auto it = seen_set.find(hash_value);
    size_t dx = (x > grid.sx)? x - grid.sx : grid.sx - x;
    size_t dy = (y > grid.sy)? y - grid.sy : grid.sy - y;
    if ( it == seen_set.end() && // 是否已经被膨胀过
      ptr_map_->getCost(x, y) != value && // 是否已经被膨胀值填充过
      cached_distance[dx][dy] < inflation_radius_) { // 是否距离小于膨胀距离
      seen_set.insert(hash_value); // 设置当前栅格为被膨胀过栅格
      ptr_map_->setCost(x, y, value); // 设置当前栅格为膨胀值
      inflation_grids[ cached_distance[dx][dy] ].push_back(GridData(hash_value, x, y, grid.sx, grid.sy)); // 添加当前栅格到优先队列中
    }
  };

  // 进行膨胀，并将膨胀区域的值置为给定值
  for (auto bin = inflation_grids.begin(); bin != inflation_grids.end(); bin++) {
    for (const auto& grid : bin->second) {
      if (grid.y + 1 < ptr_map_->getSizeInCellsY()){
        addGrid(grid, grid.x, grid.y + 1);
      }
      if (grid.y >= 1) {
        addGrid(grid, grid.x, grid.y - 1);
      }
      if (grid.x >= 1) {
        addGrid(grid, grid.x - 1, grid.y);
      }
      if (grid.x + 1 < ptr_map_->getSizeInCellsX()) {
        addGrid(grid, grid.x + 1, grid.y);
      }
    }
    (bin->second).clear();
  }

  inflation_grids.clear();
}

void MarkerMap::setStripsMarker(const Json::Value& json_file)
{
  // 判断是否已经进行了地图初始化
  if (!ptr_map_) {
    LOG(WARNING) << "marker map not initialize !";
    return;
  }

  polygons_.clear();

  if (json_file.size() == 0) {
    return;
  }
  LOG(INFO) << "polygons size " << json_file.size();

  // 获取减速带多边形顶点集合
  for (int i = 0; i < json_file.size(); i++) {
    
    if (json_file[i]["points"].size() < 3) {
      LOG(WARNING) << "strip polygon points size " << json_file[i]["points"].size() << " less than 3 !";
      continue;
    }
    LOG(INFO) << "points " << i << " size " << json_file[i]["points"].size();

    std::vector<Point2d<double>> polygon;
    for (const auto& point_json : json_file[i]["points"]) {
      LOG(INFO) << "x:" << point_json["x"].asDouble() << ",y:" << point_json["y"].asDouble();
      polygon.push_back(Point2d<double>(point_json["x"].asDouble(), point_json["y"].asDouble()));
    }

    polygons_.push_back(std::move(polygon));
  }

  // 将顶点坐标转换成顶点索引表示
  std::vector<std::vector<Point2d<size_t>>> polygons_idxs;
  polygons_idxs.reserve(polygons_.size());
  for (int i = 0; i < polygons_.size(); i++) {
    std::vector<Point2d<size_t>> idxs;
    idxs.reserve(polygons_[i].size());
    for (const auto& lp : polygons_[i]) {
      WorldmapPoint wp(lp.x, lp.y);
      CostmapPoint cp;
      if (!ptr_map_->worldToMap(wp, cp)) {
        LOG(WARNING) << "cant map wp [" << wp.d_x << ", " << wp.d_y << "] to map point !";
        idxs.clear();
        break;
      }
      idxs.push_back(std::move(Point2d<size_t>(cp.ui_x, cp.ui_y)));
    }
    if (idxs.size() < 3) continue;
    polygons_idxs.push_back(std::move(idxs));
  }

  // 判断多边形是否闭环，如果未闭环，添加起点作为最后一个点，确保闭环
  for (auto& polygon_idxs : polygons_idxs) {
    if ((polygon_idxs.front().x != polygon_idxs.back().x)
      || (polygon_idxs.front().y != polygon_idxs.back().y)) {
        polygon_idxs.push_back(polygon_idxs.front());
      }
  }

  // 标记并记录多边形的边
  std::vector<Point2d<size_t>> edge_points;
  for (auto& polygon_idxs : polygons_idxs) {
    for (int i = 0; i < polygon_idxs.size() - 1; i++) {
      // 计算边经过的栅格索引
      const auto& sp = polygon_idxs[i];
      const auto& ep = polygon_idxs[i + 1];
      std::vector<std::pair<int, int> > line_points;
      bresenham(sp.x, sp.y, ep.x, ep.y, line_points);

      // 标记栅格
      for (const auto& lp : line_points) {
        if (lp.first < 0 || lp.first >= ptr_map_->getSizeInCellsX() 
          || lp.second < 0 || lp.second >= ptr_map_->getSizeInCellsY()) {
          LOG(WARNING) << "bresenham point [" << lp.first << "," << lp.second << "] out of map border !";
          continue;
        }

        ptr_map_->setCost(lp.first, lp.second, STRIP_VALUE);
        edge_points.push_back(std::move(Point2d<size_t>(lp.first, lp.second)));
      }
    }
  }

  // 填充多边形
  for (auto& polygon_idxs : polygons_idxs) {
    // 构建多边形对象，用于判断点是否在多边形内部
    std::vector<Point2d<int>> points;
    points.reserve(polygon_idxs.size());
    for (const auto& idx : polygon_idxs) {
      points.push_back(std::move(Point2d<int>(idx.x, idx.y)));
    }
    Polygon polygoner(points);

    // 计算多边形质心，一般对于凸多边形，质心都在多边形内部
    size_t sum_x = 0;
    size_t sum_y = 0;
    for (const auto& idx : polygon_idxs) {
      sum_x += idx.x;
      sum_y += idx.y;
    }
    size_t centroid_x = sum_x / polygon_idxs.size();
    size_t centroid_y = sum_y / polygon_idxs.size();
    
    // 确定多边形内部的一个点
    if ( !polygoner.inPolygonWithoutBorder(Point2d<int>(centroid_x, centroid_y)) ) {
      LOG(INFO) << "centroid [" << centroid_x << ", " << centroid_y << "] not in polygon :(";
      // 确定多边形边界
      size_t min_x = std::numeric_limits<size_t>::max();
      size_t min_y = std::numeric_limits<size_t>::max();
      size_t max_x = 0;
      size_t max_y = 0;
      for (const auto& idx : polygon_idxs) {
        if (idx.x < min_x) min_x = idx.x;
        if (idx.x > max_x) max_x = idx.x;
        if (idx.y < min_y) min_y = idx.y;
        if (idx.y > max_y) max_y = idx.y; 
      }

      // 遍历多边形边界内的点，确定一个多边形的内点
      for (size_t x = min_x; x < max_x; x++) {
        for (size_t y = min_y; y < max_y; y++) {
          // 轮廓上的点直接跳过
          if (ptr_map_->getCost(x, y) == STRIP_VALUE) {
            continue;
          }

          if ( polygoner.inPolygonWithoutBorder(Point2d<int>(x, y)) ) {
            centroid_x = x;
            centroid_y = y;
            break;
          }
        }
      }

      LOG(WARNING) << "cant find iner point in polygon ";
      continue; // 若未找到多边形内的点，则跳过当前多边形的填充
    }
    
    // 填充多边形
    std::unordered_set<size_t> seen_set;
    std::queue<Point2d<size_t>> active_grids;
    active_grids.push(Point2d<size_t>(centroid_x, centroid_y));
    while (!active_grids.empty()) {
      const auto& grid = active_grids.front();
      ptr_map_->setCost(grid.x, grid.y, STRIP_VALUE);
      // 添加上方栅格
      if (grid.y + 1 < ptr_map_->getSizeInCellsY() &&
        ptr_map_->getCost(grid.x, grid.y + 1) != STRIP_VALUE) {
        // 判断是否已经添加到队列中
        size_t hash_value = grid.x + (grid.y + 1) * ptr_map_->getSizeInCellsX();
        auto it = seen_set.find(hash_value);
        if (seen_set.end() == it) {
          active_grids.push(Point2d<size_t>(grid.x, grid.y + 1));
          seen_set.insert(hash_value);
        }
      }
      // 添加下方栅格
      if (grid.y > 1 && 
        ptr_map_->getCost(grid.x, grid.y - 1) != STRIP_VALUE) {
        // 判断是否已经添加到队列中
        size_t hash_value = grid.x + (grid.y - 1) * ptr_map_->getSizeInCellsX();
        auto it = seen_set.find(hash_value);
        if (seen_set.end() == it) {
          active_grids.push(Point2d<size_t>(grid.x, grid.y - 1));
          seen_set.insert(hash_value);
        }
      }
      // 添加左边栅格
      if (grid.x > 1 && 
        ptr_map_->getCost(grid.x - 1, grid.y) != STRIP_VALUE) {
        // 判断是否已经添加到队列中
        size_t hash_value = (grid.x - 1) + grid.y * ptr_map_->getSizeInCellsX();
        auto it = seen_set.find(hash_value);
        if (seen_set.end() == it) {
          active_grids.push(Point2d<size_t>(grid.x - 1, grid.y));
          seen_set.insert(hash_value);
        }
      }
      // 添加右边栅格
      if (grid.x + 1 < ptr_map_->getSizeInCellsX() && 
        ptr_map_->getCost(grid.x + 1, grid.y) != STRIP_VALUE) {
        // 判断是否已经添加到队列中
        size_t hash_value = (grid.x + 1) + grid.y * ptr_map_->getSizeInCellsX();
        auto it = seen_set.find(hash_value);
        if (seen_set.end() == it) {
          active_grids.push(Point2d<size_t>(grid.x + 1, grid.y));
          seen_set.insert(hash_value);
        }
      }

      active_grids.pop();
    }
  }

  // 膨胀多边形边缘
  inflateBorder(edge_points, STRIP_VALUE);
}

MarkerType MarkerMap::getMarkerType(size_t x, size_t y)
{
  if (!ptr_map_) {
    LOG(ERROR) << "marker map no initialize, cant get marker type !";
    return MarkerType::DEFAULT;
  }

  if (ptr_map_->getCost(x, y) == STRIP_VALUE) {
    return MarkerType::STRIP;
  }

  return MarkerType::DEFAULT;
}

MarkerType MarkerMap::getMarkerType(double x, double y)
{
  if (!ptr_map_) {
    LOG(ERROR) << "marker map no initialize, cant get marker type !";
    return MarkerType::DEFAULT;
  }

  WorldmapPoint wp(x, y);
  CostmapPoint cp;
  if (!ptr_map_->worldToMap(wp, cp)) {
    LOG(WARNING) << "cant map world point [" << wp.d_x << ", " << wp.d_y << "] to marker map !";
    return MarkerType::DEFAULT;
  }

  return getMarkerType(size_t(cp.ui_x), size_t(cp.ui_y));
}

}