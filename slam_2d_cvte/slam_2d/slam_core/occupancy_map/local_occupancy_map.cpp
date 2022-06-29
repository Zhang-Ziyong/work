#include "occupancy_map/local_occupancy_map.hpp"
#include <sys/file.h>
#include "map/map_manager.hpp"
#include "common/data_struct/keyframe.hpp"

namespace cvte_lidar_slam {
std::shared_ptr<LocalOccMap> LocalOccMap::ptr_occ_map_ = nullptr;

LocalOccMap::LocalOccMap() {
  allocateMemory();

  map_array_count_ = 0;
  last_keyFrame_index_ = 0;

  ptr_map_manager_ = MapManager::getInstance();

  LOG(WARNING) << "construct new object <OccMap> successfully";
}

std::shared_ptr<LocalOccMap> LocalOccMap::getInstance() {
  if (ptr_occ_map_ == nullptr) {
    ptr_occ_map_.reset(new LocalOccMap());
  }
  return ptr_occ_map_;
}

void LocalOccMap::allocateMemory() {
  // initialize array for cmap
  map_array_ind_ = new int *[mapArrayLength];
  for (int i = 0; i < mapArrayLength; ++i)
    map_array_ind_[i] = new int[mapArrayLength];

  for (int i = 0; i < mapArrayLength; ++i)
    for (int j = 0; j < mapArrayLength; ++j) map_array_ind_[i][j] = -1;
}

bool LocalOccMap::updateLocalMap() {
  TicToc updateLocalMap_cost;
  const double &ang_min = occ_map_config_.laser_min_angle;
  // const double &ang_max = occ_map_config_.laser_max_angle;
  const double &ang_inc = occ_map_config_.laser_angle_increment;
  const double &range_max = occ_map_config_.laser_max_range;
  const double &range_min = occ_map_config_.laser_min_range;
  const double &cell_size = occ_map_config_.local_map_resolution;
  const double &inc_step = 1.0 * cell_size;

  std::vector<std::shared_ptr<KeyFrame>> surroundingKeyFrames;
  surroundingKeyFrames =
      ptr_map_manager_->DetectCandidatesByTime(occ_map_config_.local_map_size);
  if (surroundingKeyFrames.empty()) {
    return false;
  }
  size_t cur_index =
      surroundingKeyFrames[surroundingKeyFrames.size() - 1]->index_;
  if (cur_index <= last_keyFrame_index_) {
    return false;
  }
  clearTmpMap();
  last_keyFrame_index_ = cur_index;
  for (unsigned int i = 0; i < surroundingKeyFrames.size(); i++) {
    Mat34d frame_pos;
    frame_pos = surroundingKeyFrames[i]->getPose();
    Pose2d laser_pose = Mathbox::Mat34d2Pose2d(frame_pos);
    if (!cloud2LaserScan(surroundingKeyFrames[i]->without_ground_cloud_)) {
      LOG(WARNING) << "Empty obstacle_cloud";
      return false;
    }

    Vec2d last_grid(Eigen::Infinity, Eigen::Infinity);

    /* for every laser beam */
    int count = 0;
    for (size_t j = 0; j < laser_scan_.size(); j++) {
      /* 获取当前beam的距离 */
      const double &R = laser_scan_[j];
      if (R > range_max || R < range_min)
        continue;

      /* 沿着激光射线以inc_step步进，更新地图*/
      double angle = ang_inc * j + ang_min;
      // TODO:是否需要构造一个表，直接lookup table
      double cangle = cos(angle);
      double sangle = sin(angle);

      for (double r = 0; r < R + cell_size; r += inc_step) {
        Vec2d p_l(r * cangle, r * sangle);  //在激光雷达坐标系下的坐标
        /* 转换到世界坐标系下 */
        Vec2d p_w = laser_pose * p_l;
        double inv_model = laserInvModel(r, R, cell_size);
        const double diff = (p_w - last_grid).norm();
        if (fabs(diff) < 0.05) {
          continue;
        }
        ++count;
        updateChildMap(p_w, inv_model);
        last_grid = p_w;
      }
    }
    std::fill(laser_scan_.begin(), laser_scan_.end(),
              occ_map_config_.laser_max_range + 1.0);

    int pos_max_cube_x = 0;
    int pos_max_cube_y = 0;
    int neg_max_cube_x = 0;
    int neg_max_cube_y = 0;
    for (unsigned int i = 0; i < getMapArraySize(); i++) {
      if (getMapArrayIndX(i) - rootCubeIndex > 0) {
        pos_max_cube_x =
            std::max(pos_max_cube_x, getMapArrayIndX(i) - rootCubeIndex);
      } else {
        neg_max_cube_x =
            std::max(neg_max_cube_x, abs(getMapArrayIndX(i) - rootCubeIndex));
      }

      if (getMapArrayIndY(i) - rootCubeIndex > 0) {
        pos_max_cube_y =
            std::max(pos_max_cube_y, getMapArrayIndY(i) - rootCubeIndex);
      } else {
        neg_max_cube_y =
            std::max(neg_max_cube_y, abs(getMapArrayIndY(i) - rootCubeIndex));
      }
    }
    int map_width = (pos_max_cube_x + neg_max_cube_x) * mapCubeArrayLength;
    int map_height = (pos_max_cube_y + neg_max_cube_y) * mapCubeArrayLength;
    if (map_width <= 0 || map_height <= 0) {
      return false;
    }

    local_occ_map_.info.width = map_width;
    local_occ_map_.info.height = map_height;
    local_occ_map_.info.resolution = localMapResolution;
    local_occ_map_.data.clear();
    local_occ_map_.data.resize(map_width * map_height);
    std::fill(local_occ_map_.data.begin(), local_occ_map_.data.end(), -1);

    // local map origin x and y
    local_map_origin_point_.x = 0 - neg_max_cube_x * mapCubeLength;
    local_map_origin_point_.y = 0 - neg_max_cube_y * mapCubeLength;
    // local_map_origin_point_.z = origin_robot_point_.z;

    // update global mao origin
    local_occ_map_.info.origin(0, 3) = local_map_origin_point_.x;
    local_occ_map_.info.origin(1, 3) = local_map_origin_point_.y;

    // local map origin cube id (in global map)
    local_map_origin_grid_.cubeX =
        int((local_map_origin_point_.x + mapCubeLength / 2.0) / mapCubeLength) +
        rootCubeIndex;
    local_map_origin_grid_.cubeY =
        int((local_map_origin_point_.y + mapCubeLength / 2.0) / mapCubeLength) +
        rootCubeIndex;
    if (local_map_origin_point_.x + mapCubeLength / 2.0 < 0)
      --local_map_origin_grid_.cubeX;
    if (local_map_origin_point_.y + mapCubeLength / 2.0 < 0)
      --local_map_origin_grid_.cubeY;
    // local map origin grid id (in sub-map)
    float originCubeOriginX,
        originCubeOriginY;  // the orign of submap that the local map origin
    // belongs to (note the submap may not be created yet,
    // cannot use originX and originY)
    originCubeOriginX =
        (local_map_origin_grid_.cubeX - rootCubeIndex) * mapCubeLength -
        mapCubeLength / 2.0;
    originCubeOriginY =
        (local_map_origin_grid_.cubeY - rootCubeIndex) * mapCubeLength -
        mapCubeLength / 2.0;
    local_map_origin_grid_.gridX = int(
        (local_map_origin_point_.x - originCubeOriginX) / localMapResolution);
    local_map_origin_grid_.gridY = int(
        (local_map_origin_point_.y - originCubeOriginY) / localMapResolution);

    for (int x = 0; x < map_width; x++) {
      for (int y = 0; y < map_height; y++) {
        int indX = local_map_origin_grid_.gridX + x;
        int indY = local_map_origin_grid_.gridY + y;

        Grid thisGrid;

        thisGrid.cubeX =
            local_map_origin_grid_.cubeX + indX / mapCubeArrayLength;
        thisGrid.cubeY =
            local_map_origin_grid_.cubeY + indY / mapCubeArrayLength;

        thisGrid.gridX = indX % mapCubeArrayLength;
        thisGrid.gridY = indY % mapCubeArrayLength;

        thisGrid.mapID = map_array_ind_[thisGrid.cubeX][thisGrid.cubeY];

        // if sub-map is not created yet
        if (!subMapExist(thisGrid.cubeX, thisGrid.cubeY)) {
          continue;
        }

        MapCell *thisCell = grid2MapCell(thisGrid);
        // skip unknown grid
        if (nullptr != thisCell && fabs(thisCell->log_odds) > 0.01) {
          int index = x + y * map_width;  // index of the 1-D array
          local_occ_map_.data[index] = (int8_t) (100 * thisCell->getBel());
        }
      }
    }
  }
  int frame_size = surroundingKeyFrames.size();
  int update_index = (frame_size - 1) / 2;

  for (int i = frame_size - 1; i >= update_index; --i) {
    if (i < 0 || i > frame_size - 1) {
      break;
    }
    Mat34d frame_pos;
    frame_pos = surroundingKeyFrames[i]->getPose();

    laserCloud::Ptr surround_corner_cloud_tmp(new laserCloud());
    for (int j = 0; j < (surroundingKeyFrames[i]->corner_cloud_)->size(); j++) {
      PointType pointIn = (surroundingKeyFrames[i]->corner_cloud_)->points[j];
      PointType pointTo;
      Transformbox::pointAssociateToMap(&pointIn, &pointTo, frame_pos);
      auto occ_value = ptr_occ_map_->getPointOccupied(pointTo);
      if (occ_value >= 0 && occ_value <= 40) {
        continue;
      }
      surround_corner_cloud_tmp->push_back(pointIn);
    }
    surroundingKeyFrames[i]->corner_cloud_ = surround_corner_cloud_tmp;
  }
  float update_map_cost = updateLocalMap_cost.toc();
  LOG(INFO) << "LocalMap - update_time: " << update_map_cost << " ms ;";
  // saveMapFile(local_occ_map_, "/home/caoyong/map/");

  return true;
}

int8_t LocalOccMap::getPointOccupied(const PointType &point_in) {
  Grid thisGrid;
  Vec2d point_grid(point_in.x, point_in.y);
  if (!getPointGrid(point_grid, thisGrid)) {
    return -1;
  }
  // Get current cell pointer
  MapCell *thisCell = grid2MapCell(thisGrid);
  if (nullptr == thisCell) {
    return -1;
  }

  return (int8_t) (100 * thisCell->getBel());
}

double LocalOccMap::laserInvModel(const double &r, const double &R,
                                  const double &cell_size) {
  // double P_occ = 0.6;
  // double P_free = 0.4;
  // double P_prior = 0.5;
  if (r < (R - 0.5 * cell_size))
    return -0.3;  // P_free;

  if (r > (R + 0.5 * cell_size))
    return 0;  //对数概率　P_prior;

  // return 0.2;  // P_occ;
  return 0.5;  // P_occ;
}
void LocalOccMap::updateChildMap(const Vec2d &point,
                                 const double laser_inv_value) {
  // Find point index in global map
  Grid thisGrid;
  if (!getPointGrid(point, thisGrid)) {
    return;
  }
  // Get current cell pointer
  MapCell *thisCell = grid2MapCell(thisGrid);
  if (nullptr == thisCell) {
    return;
  }

  // update occupancy
  thisCell->setLogBel(laser_inv_value, 100);  // 更新
}

bool LocalOccMap::getPointGrid(const Vec2d &point, Grid &thisGrid) {
  thisGrid.cubeX =
      int((point[0] + mapCubeLength / 2.0) / mapCubeLength) + rootCubeIndex;
  thisGrid.cubeY =
      int((point[1] + mapCubeLength / 2.0) / mapCubeLength) + rootCubeIndex;
  if (point[0] + mapCubeLength / 2.0 < 0.) {
    --thisGrid.cubeX;
  }
  if (point[1] + mapCubeLength / 2.0 < 0.) {
    --thisGrid.cubeY;
  }

  // Decide whether a point is out of pre-allocated map
  if (thisGrid.cubeX >= 0 && thisGrid.cubeX < mapArrayLength &&
      thisGrid.cubeY >= 0 && thisGrid.cubeY < mapArrayLength) {
    // Point is in the boundary, but this sub-map is not allocated before
    // Allocate new memory for this sub-map and save it to mapArray
    if (map_array_ind_[thisGrid.cubeX][thisGrid.cubeY] == -1) {
      map_array_.emplace_back(map_array_count_, thisGrid.cubeX, thisGrid.cubeY);
      map_array_ind_[thisGrid.cubeX][thisGrid.cubeY] = map_array_count_;
      ++map_array_count_;
    }
  } else {
    LOG(ERROR) << "Point cloud is out of elevation map boundary. Change params "
                  "->mapArrayLength<-. The program will crash!";
    return false;
  }
  // sub-map id
  thisGrid.mapID = map_array_ind_[thisGrid.cubeX][thisGrid.cubeY];
  // Find the index for this point in this sub-map (grid index)
  thisGrid.gridX = (int) ((point[0] - map_array_[thisGrid.mapID].originX) /
                          localMapResolution);
  thisGrid.gridY = (int) ((point[1] - map_array_[thisGrid.mapID].originY) /
                          localMapResolution);
  if (thisGrid.gridX < 0 || thisGrid.gridY < 0 ||
      thisGrid.gridX >= mapCubeArrayLength ||
      thisGrid.gridY >= mapCubeArrayLength) {
    return false;
  }
  return true;
}

MapCell *LocalOccMap::grid2MapCell(const Grid &grid) {
  MapCell *p_cell = nullptr;
  try {
    p_cell =
        &(map_array_.at(grid.mapID).cellArray.at(grid.gridX).at(grid.gridY));
  } catch (...) {
    LOG(ERROR) << grid.mapID << " " << grid.gridX << " " << grid.gridY
               << std::endl;
  }
  return p_cell;
}

bool LocalOccMap::cloud2LaserScan(const laserCloud::Ptr cloud) {
  // convert point to scan
  int cloudSize = cloud->points.size();
  if (cloudSize < 10) {
    return false;
  }
  for (int i = 0; i < cloudSize; ++i) {
    PointType point = cloud->points[i];
    float x = point.x;
    float y = point.y;
    float range = std::sqrt(x * x + y * y);

    float angle = std::atan2(y, x);
    if (range > occ_map_config_.laser_max_range || point.z > 0.9) {
      continue;
    }
    uint index = std::round((angle - occ_map_config_.laser_min_angle) /
                            occ_map_config_.laser_angle_increment);
    if (index >= 0 && index < range_size_) {
      laser_scan_[index] = std::min(laser_scan_[index], range);
    }
  }
  return true;
}

void LocalOccMap::saveMapFile(const UserOccupancyGrid &map,
                              const std::string &map_dir) {
  LOG(INFO) << "Received a " << map.info.width << "X" << map.info.height
            << "map " << map.info.resolution << "m/pix";
  const int8_t &threshold_occupied = 60;
  const int8_t &threshold_free = 40;
  std::string mapdatafile = map_dir + "map_2d.pgm";

  LOG(INFO) << "Writing map occupancy data to " << mapdatafile;
  FILE *out = fopen(mapdatafile.c_str(), "w");
  if (!out) {
    LOG(ERROR) << "Couldn't save map file to " << mapdatafile;
    return;
  }

  fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
          map.info.resolution, map.info.width, map.info.height);
  for (unsigned int y = 0; y < map.info.height; y++) {
    for (unsigned int x = 0; x < map.info.width; x++) {
      unsigned int i = x + (map.info.height - y - 1) * map.info.width;
      if (map.data[i] >= 0 && map.data[i] <= threshold_free) {
        // [0,free)
        fputc(254, out);
      } else if (map.data[i] >= threshold_occupied) {
        // (occ,255]
        fputc(000, out);
      } else {
        //  occ[0.25,0.65]
        fputc(205, out);
      }
    }
  }

  fclose(out);

  std::string mapmetadatafile = map_dir + "map_2d.yaml";
  LOG(INFO) << "Writing map occupancy data to " << mapmetadatafile;
  FILE *yaml = fopen(mapmetadatafile.c_str(), "w");
  flock(yaml->_fileno, LOCK_EX);

  Pose2d laser_pose = Mathbox::Mat34d2Pose2d(map.info.origin);

  // TODO: use ours Math tools
  fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\n",
          mapdatafile.c_str(), map.info.resolution, laser_pose.x(),
          laser_pose.y(), laser_pose.yaw());
  fprintf(yaml, "height: %d\nwidth: %d\n", map.info.height, map.info.width);
  fprintf(yaml, "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n");

  flock(yaml->_fileno, LOCK_UN);
  fclose(yaml);
}

}  // namespace cvte_lidar_slam