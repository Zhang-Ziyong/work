/*
 * @Author: your name
 * @Date: 2021-09-28 16:36:55
 * @LastEditTime: 2021-09-28 16:36:56
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /navigation/navigation/clean_decision/src/clean_decision.cpp
 */
#include <map>
#include <set>
#include <unordered_set>
#include <utility>
#include "glog/logging.h"
#include "clean_decision.hpp"
#include "data_base/obstacle_map.hpp"
#include "fcp_planner/fcp_path.hpp"
#include "navigation_mediator.hpp"
using namespace full_coverage_planner;
// #include "bonsCCPP.h"
namespace CVTE_BABOT {
CleanDecision::CleanDecision() {
  init_pose_ = false;
  init_map_ = false;
  mission_type_ = ActionType::IDEL;
  last_pose_.setZero();
  ptr_obstacle_map_ = std::make_shared<ObstacleMap>();
  calcuMarkArea();
}
CleanDecision::CleanDecision(const CleanConfig &config) {
  init_pose_ = false;
  init_map_ = false;
  config_ = config;
  mission_type_ = ActionType::IDEL;
  last_pose_.setZero();
  ptr_obstacle_map_ = std::make_shared<ObstacleMap>();
  calcuMarkArea();
}

void CleanDecision::reset() {
  init_pose_ = false;
  init_map_ = false;
  mission_type_ = ActionType::IDEL;
  if (ptr_obstacle_map_ == nullptr) {
    LOG(INFO) << "not init ptr_obstacle_map_";
    return;
  }
  ptr_obstacle_map_->resetMaps();
  multi_area_.clear();
}

void CleanDecision::updateAlreadyCleanArea(const Eigen::Vector3d &cur_pose) {
  if (ptr_obstacle_map_ == nullptr) {
    LOG(INFO) << "not init ptr_obstacle_map_";
    return;
  }
  if (mission_type_ == ActionType::CLEAN) {
    if (!init_pose_) {
      last_pose_ = cur_pose;
      init_pose_ = true;
    } else if ((cur_pose - last_pose_).norm() > config_.update_thresh) {
      last_pose_ = cur_pose;
      Eigen::Matrix<double, 2, 3> T_wb;
      T_wb << cos(cur_pose(2)), -sin(cur_pose(2)), cur_pose(0),
          sin(cur_pose(2)), cos(cur_pose(2)), cur_pose(1);
      for (size_t index = 0; index < mark_area_.size(); index++) {
        Eigen::Vector2d mark_point =
            T_wb.block<2, 2>(0, 0) * mark_area_[index] + T_wb.block<2, 1>(0, 2);
        ptr_obstacle_map_->setWorldPointValue(
            GlobalmapPoint(mark_point(0), mark_point(1)), 254);
      }
    }
  }
}

bool CleanDecision::setCleanMap(const std::string &map_path) {
  if (ptr_obstacle_map_ == nullptr) {
    LOG(INFO) << "not init ptr_obstacle_map_";
    return false;
  }
  if (!init_map_) {
    ptr_obstacle_map_->readMap(map_path);
    init_map_ = true;
  } else {
    LOG(INFO) << "alread read map";
  }
  return true;
}

bool CleanDecision::saveAlreadyCleanMap(const std::string &map_path) {
  if (ptr_obstacle_map_ == nullptr) {
    LOG(INFO) << "not init ptr_obstacle_map_";
    return false;
  }
  return ptr_obstacle_map_->saveMap(map_path);
}

bool CleanDecision::AdditionalCoverPlan(const std::string &map_path,
                                        const MutilArea &multi_area,
                                        SubPath &path) {
  LOG(INFO) << "multi_area size: " << multi_area.size();
  if (ptr_obstacle_map_ == nullptr) {
    LOG(ERROR) << "not init ptr_obstacle_map_";
    return false;
  } else if (ptr_obstacle_map_->getSizeInCellsX() == 0) {
    LOG(ERROR) << "no map";
    return false;
  }
  std::vector<std::vector<GridmapPoint>> multi_area_points;
  for (size_t index = 0; index < multi_area.size(); index++) {
    std::vector<GridmapPoint> area_points = getAreaPoint(multi_area[index]);
    multi_area_points.push_back(area_points);
  }
  std::vector<std::vector<GridmapPoint>> cluster_area;
  clusterUncleanArea(multi_area_points, cluster_area);
  LOG(INFO) << "cluster_area size " << cluster_area.size();
  std::vector<std::vector<AreaVertex>> bons_areas;
  std::vector<AreaVertex> bons_area_points;
  for (size_t i = 0; i < cluster_area.size(); i++) {
    bons_area_points.clear();
    LOG(INFO) << "cluster_area " << i << " size " << cluster_area[i].size();
    //小于四百个栅格的区域不补扫
    if (cluster_area[i].size() < 400) {
      continue;
    }
    std::vector<GridmapPoint> min_rect;
    int mini_rect_area;
    minimumEncloseRect(cluster_area[i], min_rect, mini_rect_area);
    if (min_rect.size() != 4) {
      LOG(INFO) << "no clean rectangle";
      continue;
    }
    if (mini_rect_area > 3 * cluster_area[i].size()) {
      LOG(INFO) << "the unclean area in rectangle is too small";
      continue;
    }
    unsigned int max_ix = 0;
    unsigned int min_ix = INT_MAX;
    unsigned int max_iy = 0;
    unsigned int min_iy = INT_MAX;
    //调试使用
    // for (size_t j = 0; j < min_rect.size(); j++) {
    //   unsigned int ix, iy;
    //   ix = min_rect[j](0);
    //   iy = min_rect[j](1);
    //   max_ix = ix > max_ix ? ix : max_ix;
    //   min_ix = ix < min_ix ? ix : min_ix;
    //   max_iy = iy > max_iy ? iy : max_iy;
    //   min_iy = iy < min_iy ? iy : min_iy;
    // }
    // for (size_t j = min_ix; j <= max_ix; j++) {
    //   ptr_obstacle_map_->setMapPointValue(GridmapPoint(j, min_iy), 0);
    //   ptr_obstacle_map_->setMapPointValue(GridmapPoint(j, max_iy), 0);
    // }
    // for (size_t j = min_iy; j <= max_iy; j++) {
    //   ptr_obstacle_map_->setMapPointValue(GridmapPoint(min_ix, j), 0);
    //   ptr_obstacle_map_->setMapPointValue(GridmapPoint(max_ix, j), 0);
    // }

    GlobalmapPoint point1, point2, point3, point4;
    ptr_obstacle_map_->mapToWorld(min_rect[0], point1);
    ptr_obstacle_map_->mapToWorld(min_rect[1], point2);
    ptr_obstacle_map_->mapToWorld(min_rect[2], point3);
    ptr_obstacle_map_->mapToWorld(min_rect[3], point4);
    bons_area_points.push_back(AreaVertex(point1(0), point1(1)));
    bons_area_points.push_back(AreaVertex(point2(0), point2(1)));
    bons_area_points.push_back(AreaVertex(point3(0), point3(1)));
    bons_area_points.push_back(AreaVertex(point4(0), point4(1)));
    bons_areas.push_back(bons_area_points);
  }
  return generaCoverPath(bons_areas, map_path, path);
}

bool CleanDecision::generaCoverPath(
    const std::vector<std::vector<AreaVertex>> &bons_areas,
    const std::string &map_path, SubPath &path) const {
  if (!bons_areas.empty()) {
    LOG(INFO) << "start additional coverage plan";
    LOG(INFO) << "map path: " << map_path;
    FCPPath ccpp;
    FullCoveragePlanerParams bons_config;
    std::shared_ptr<NavigationMediator> ptr_navigation_mediator =
        NavigationMediator::getPtrInstance();
    if (!ptr_navigation_mediator->isParameterReady()) {
      LOG(ERROR) << "Is mediator's Parameter initialized? ";
      return false;
    }
    ptr_navigation_mediator->getParam("addition_clean.changeColosCost",
                                      bons_config.changecost, 90.0);
    ptr_navigation_mediator->getParam("addition_clean.turn_offset",
                                      bons_config.turn_offset, 0.9);
    ptr_navigation_mediator->getParam("addition_clean.obs_offset",
                                      bons_config.obs_offset, 0.5);
    ptr_navigation_mediator->getParam("addition_clean.inflation_dis",
                                      bons_config.inflation_dis, 0.7);
    ptr_navigation_mediator->getParam("addition_clean.dis_of_path",
                                      bons_config.dis_of_path, 0.7);

    LOG(INFO) << "additional_clean params: ";
    LOG(INFO) << "addition_clean.changeColosCost: " << bons_config.changecost;
    LOG(INFO) << "addition_clean.turn_offset: " << bons_config.turn_offset;
    LOG(INFO) << "addition_clean.obs_offset: " << bons_config.obs_offset;
    LOG(INFO) << "addition_clean.inflation_dis: " << bons_config.inflation_dis;
    LOG(INFO) << "addition_clean.dis_of_path: " << bons_config.dis_of_path;

    std::vector<Point> path_points;
    std::vector<std::vector<PointMap>> supplemental_path;
    if (ccpp.readMap(map_path)) {
      LOG(INFO) << "map ready";
      // ccpp.addWelts(welts);
      ccpp.generateCoveragePathByAreas(path_points, bons_areas,
                                       supplemental_path, bons_config);
      ccpp.savaAsImage(path_points, "/tmp/ccpp.png");
      for (size_t index = 0; index < path_points.size(); index++) {
        path.wps.push_back(Pose2d(path_points[index].x, path_points[index].y,
                                  path_points[index].yaw));
        path.wpis.push_back(WayPointInfo(0.0, 0.0, 0.0));
      }
      return true;
    } else {
      LOG(ERROR) << "error when read map";
      return false;
    }
  }
  return false;
  LOG(INFO) << "additional cover plan done";
}

void CleanDecision::minimumEncloseRect(
    const std::vector<GridmapPoint> &area,
    std::vector<GridmapPoint> &mini_rectangle, int &mini_area) const {
  mini_area = INT_MAX;
  mini_rectangle.clear();
  int degree_resolution = 10;
  int range = 180;
  int count = range / degree_resolution;
  for (int i = 0; i < count; i++) {
    double rot_angle = M_PI * i / count;
    // LOG(INFO) << "rotate angle: " << rot_angle;
    Eigen::Matrix<double, 2, 2> trans_mat;
    trans_mat << cos(rot_angle), -sin(rot_angle), sin(rot_angle),
        cos(rot_angle);
    double max_ix = -INT_MAX;
    double min_ix = INT_MAX;
    double max_iy = -INT_MAX;
    double min_iy = INT_MAX;
    for (size_t j = 0; j < area.size(); j++) {
      auto d_point = trans_mat * area[j].cast<double>();
      double ix, iy;
      ix = d_point(0);
      iy = d_point(1);
      max_ix = ix > max_ix ? ix : max_ix;
      min_ix = ix < min_ix ? ix : min_ix;
      max_iy = iy > max_iy ? iy : max_iy;
      min_iy = iy < min_iy ? iy : min_iy;
    }
    std::vector<GridmapPoint> temp_rect;
    Eigen::Vector2d point1(min_ix, min_iy);
    Eigen::Vector2d point2(max_ix, min_iy);
    Eigen::Vector2d point3(max_ix, max_iy);
    Eigen::Vector2d point4(min_ix, max_iy);
    Eigen::Vector2d point = trans_mat.transpose() * point1;
    temp_rect.push_back(
        GridmapPoint(static_cast<int>(point(0)), static_cast<int>(point(1))));
    point = trans_mat.transpose() * point2;
    temp_rect.push_back(
        GridmapPoint(static_cast<int>(point(0)), static_cast<int>(point(1))));
    point = trans_mat.transpose() * point3;
    temp_rect.push_back(
        GridmapPoint(static_cast<int>(point(0)), static_cast<int>(point(1))));
    point = trans_mat.transpose() * point4;
    temp_rect.push_back(
        GridmapPoint(static_cast<int>(point(0)), static_cast<int>(point(1))));
    int rect_area = (temp_rect[1] - temp_rect[0]).norm() *
                    (temp_rect[2] - temp_rect[1]).norm();
    if (rect_area < mini_area) {
      mini_area = rect_area;
      mini_rectangle = temp_rect;
    }
  }
  LOG(INFO) << "mini_area: " << mini_area;
}

void CleanDecision::clusterUncleanArea(
    const std::vector<std::vector<GridmapPoint>> &multi_area_points,
    std::vector<std::vector<GridmapPoint>> &cluster_area_points) {
  LOG(INFO) << "multi_area_size: " << multi_area_points.size();
  cluster_area_points.clear();
  std::unordered_set<int> unclean_pt_hash;
  for (size_t index = 0; index < multi_area_points.size(); index++) {
    std::vector<GridmapPoint> area_points = multi_area_points[index];
    for (size_t i = 0; i < area_points.size(); i++) {
      if (ptr_obstacle_map_->getMapPointValue(area_points[i]) < 200) {
        unclean_pt_hash.insert(
            ptr_obstacle_map_->getIndex(area_points[i](0), area_points[i](1)));
      }
    }
  }
  //聚类
  // std::vector<std::vector<int>> cluster_area;
  while (!unclean_pt_hash.empty()) {
    auto it = unclean_pt_hash.begin();
    std::vector<GridmapPoint> area;
    std::unordered_set<int> close_set;
    std::queue<int> open_set;
    open_set.push(*it);
    while (!open_set.empty()) {
      int index = open_set.front();
      int n_left = index - 1;
      int n_right = index + 1;
      int n_top = index - ptr_obstacle_map_->getSizeInCellsX();
      int n_down = index + ptr_obstacle_map_->getSizeInCellsX();
      if (close_set.find(n_left) == close_set.end() &&
          unclean_pt_hash.find(n_left) != unclean_pt_hash.end()) {
        open_set.push(n_left);
        close_set.insert(n_left);
      }
      if (close_set.find(n_right) == close_set.end() &&
          unclean_pt_hash.find(n_right) != unclean_pt_hash.end()) {
        open_set.push(n_right);
        close_set.insert(n_right);
      }
      if (close_set.find(n_top) == close_set.end() &&
          unclean_pt_hash.find(n_top) != unclean_pt_hash.end()) {
        open_set.push(n_top);
        close_set.insert(n_top);
      }
      if (close_set.find(n_down) == close_set.end() &&
          unclean_pt_hash.find(n_down) != unclean_pt_hash.end()) {
        open_set.push(n_down);
        close_set.insert(n_down);
      }
      open_set.pop();
      unclean_pt_hash.erase(index);
      close_set.insert(index);
      unsigned int ui_mx, ui_my;
      ptr_obstacle_map_->indexToCells(index, ui_mx, ui_my);
      area.push_back(GridmapPoint(ui_mx, ui_my));
    }
    cluster_area_points.push_back(area);
  }
}

std::vector<GridmapPoint> CleanDecision::getAreaPoint(const Area &area) const {
  if (ptr_obstacle_map_ == nullptr) {
    LOG(ERROR) << "not init ptr_obstacle_map";
    return std::vector<GridmapPoint>();
  }
  std::vector<GridmapPoint> grid_area;
  for (size_t index = 0; index < area.size(); index++) {
    GridmapPoint grid_point;
    if (ptr_obstacle_map_->worldToMap(area[index], grid_point)) {
      grid_area.push_back(grid_point);
      LOG(INFO) << "grid_point " << grid_point(0) << " " << grid_point(1);
    }
  }
  std::vector<std::pair<GridmapPoint, GridmapPoint>> point_pairs;
  unsigned int min_ix = ptr_obstacle_map_->getSizeInCellsX();
  unsigned int max_ix = 0;
  for (size_t index = 0; index < grid_area.size(); index++) {
    min_ix = grid_area[index](0) < min_ix ? grid_area[index](0) : min_ix;
    max_ix = grid_area[index](0) > max_ix ? grid_area[index](0) : max_ix;
    if (index != grid_area.size() - 1) {
      std::pair<GridmapPoint, GridmapPoint> point_pair(grid_area[index],
                                                       grid_area[index + 1]);
      point_pairs.push_back(point_pair);
    } else {
      std::pair<GridmapPoint, GridmapPoint> point_pair(grid_area[index],
                                                       grid_area[0]);
      point_pairs.push_back(point_pair);
    }
  }
  LOG(INFO) << "min_ix: " << min_ix << " max_ix: " << max_ix;
  LOG(INFO) << "point_pairs size: " << point_pairs.size();
  std::vector<GridmapPoint> area_points;
  size_t shrink_size = 20;  //内缩0.5m
  for (size_t ix = min_ix + shrink_size; ix < max_ix - shrink_size; ix++) {
    std::shared_ptr<std::pair<GridmapPoint, GridmapPoint>> ptr_pair1 = nullptr;
    std::shared_ptr<std::pair<GridmapPoint, GridmapPoint>> ptr_pair2 = nullptr;
    for (size_t j = 0; j < point_pairs.size(); j++) {
      if ((point_pairs[j].first(0) > ix && point_pairs[j].second(0) <= ix) ||
          (point_pairs[j].first(0) <= ix && point_pairs[j].second(0) > ix)) {
        if (ptr_pair1 == nullptr) {
          ptr_pair1 = std::make_shared<std::pair<GridmapPoint, GridmapPoint>>(
              point_pairs[j]);
        } else if (ptr_pair2 == nullptr) {
          ptr_pair2 = std::make_shared<std::pair<GridmapPoint, GridmapPoint>>(
              point_pairs[j]);
        } else {
          break;
        }
      }
    }
    // LOG(INFO) << "pair1 1 " << ptr_pair1->first(0) << " " <<
    // ptr_pair1->first(1)
    //           << " 2 " << ptr_pair1->second(0) << " " <<
    //           ptr_pair1->second(1);
    // LOG(INFO) << "pair1 2 " << ptr_pair2->first(0) << " " <<
    // ptr_pair2->first(1)
    //           << " 2 " << ptr_pair2->second(0) << " " <<
    //           ptr_pair2->second(1);
    if (ptr_pair1 != nullptr && ptr_pair2 != nullptr) {
      double temp_ix = ix;
      double pair1_temp_ix1 = ptr_pair1->first(0);
      double pair1_temp_iy1 = ptr_pair1->first(1);
      double pair1_temp_ix2 = ptr_pair1->second(0);
      double pair1_temp_iy2 = ptr_pair1->second(1);
      double pair2_temp_ix1 = ptr_pair2->first(0);
      double pair2_temp_iy1 = ptr_pair2->first(1);
      double pair2_temp_ix2 = ptr_pair2->second(0);
      double pair2_temp_iy2 = ptr_pair2->second(1);
      //处理临界情况
      if (ptr_pair1->first(0) == ptr_pair1->second(0)) {
        pair1_temp_ix2 += 1;
      }
      if (ptr_pair2->first(0) == ptr_pair2->second(0)) {
        pair2_temp_ix2 += 1;
      }
      int iy1 = pair1_temp_iy1 + ((temp_ix - pair1_temp_ix1) /
                                  (pair1_temp_ix2 - pair1_temp_ix1)) *
                                     (pair1_temp_iy2 - pair1_temp_iy1);
      int iy2 = pair2_temp_iy1 + ((temp_ix - pair2_temp_ix1) /
                                  (pair2_temp_ix2 - pair2_temp_ix1)) *
                                     (pair2_temp_iy2 - pair2_temp_iy1);
      unsigned int min_iy, max_iy;
      if (iy1 > iy2) {
        min_iy = iy2;
        max_iy = iy1;
      } else {
        min_iy = iy1;
        max_iy = iy2;
      }
      // LOG(INFO) << "iy1: " << iy1 << " iy2: " << iy2;
      // LOG(INFO) << "min_iy: " << min_iy << " max_iy:" << max_iy;
      for (size_t iy = min_iy + shrink_size; iy <= max_iy - shrink_size; iy++) {
        area_points.push_back(GridmapPoint(ix, iy));
        // ptr_obstacle_map_->setMapPointValue(GridmapPoint(ix, iy), 254);
      }
    }
  }
  LOG(INFO) << "get area point done";
  return area_points;
}

void CleanDecision::calcuMarkArea() {
  double inc_x = 0.03;
  double inc_y = 0.03;
  for (double x = -config_.clean_length; x < 0; x += inc_x) {
    for (double y = -config_.clean_width / 2.0; y < config_.clean_width / 2.0;
         y += inc_y) {
      mark_area_.push_back(Eigen::Vector2d(x, y));
    }
  }
}

}  // namespace CVTE_BABOT