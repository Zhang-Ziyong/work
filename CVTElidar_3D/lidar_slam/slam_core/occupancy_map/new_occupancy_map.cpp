#include "occupancy_map/new_occupancy_map.hpp"
#include <sys/file.h>
#include "map/map_manager.hpp"
#include "common/debug_tools/tic_toc.h"
#include "common/math_base/slam_transform.hpp"

namespace cvte_lidar_slam {

std::shared_ptr<OccMap> OccMap::ptr_occ_map_ = nullptr;

OccMap::OccMap() {
  default_map_filepath_ = "./";

  allocateMemory();

  map_array_count_ = 0;

  ptr_map_manager_ = MapManager::getInstance();

  LOG(WARNING) << "construct new object <OccMap> successfully";
}

std::shared_ptr<OccMap> OccMap::getInstance() {
  if (ptr_occ_map_ == nullptr) {
    ptr_occ_map_.reset(new OccMap());
  }
  return ptr_occ_map_;
}

void OccMap::allocateMemory() {
  // initialize array for cmap
  downSize_filter_.setLeafSize(0.3, 0.3, 0.3);
  cloud_curb_.reset(new laserCloud());
  cloud_view_.reset(new laserCloud());
  cloud_range_.reset(new laserCloud());
  cloud_trans_.reset(new laserCloud());
  map_array_ind_ = new int *[mapArrayLength];
  for (int i = 0; i < mapArrayLength; ++i)
    map_array_ind_[i] = new int[mapArrayLength];

  for (int i = 0; i < mapArrayLength; ++i)
    for (int j = 0; j < mapArrayLength; ++j) map_array_ind_[i][j] = -1;

  init_flag_.resize(occ_map_config_.curb_map_length);
  for (int i = 0; i < occ_map_config_.curb_map_length; ++i) {
    init_flag_[i].resize(occ_map_config_.curb_map_length);
  }
  min_height_.resize(occ_map_config_.curb_map_length);
  for (int i = 0; i < occ_map_config_.curb_map_length; ++i) {
    min_height_[i].resize(occ_map_config_.curb_map_length);
  }
  max_height_.resize(occ_map_config_.curb_map_length);
  for (int i = 0; i < occ_map_config_.curb_map_length; ++i) {
    max_height_[i].resize(occ_map_config_.curb_map_length);
  }
}

void OccMap::insertObstacleCloud(const laserCloud::Ptr obstacle_cloud,
                                 const Mat34d &pose, const size_t frame_id,
                                 const Mat34d &delta_pose) {
  has_robot_point_ = true;

  cloud_trans_ = Transformbox::transformPointCloud(
      Mathbox::inversePose34d(T_lo_), obstacle_cloud);
  Mat34d base_delta_pose = Mathbox::multiplePose34d(
      Mathbox::multiplePose34d(Mathbox::inversePose34d(T_lo_), delta_pose),
      T_lo_);
  /* 获取激光的信息 */
  ScanData scan_data;
  scan_data.delta_pose = base_delta_pose;
  scan_data.frame_id = frame_id;

  if (!cloud2LaserScan(cloud_trans_)) {
    LOG(WARNING) << "Empty obstacle_cloud";
    return;
  }

  if (laser_scan_.empty()) {
    LOG(WARNING) << "Empty laser scan";
    return;
  }
  scan_data.v_laser_scan.reserve(laser_scan_.size());
  scan_data.v_laser_scan = laser_scan_;

  if (occ_map_config_.use_curb_cloud || occ_map_config_.use_view_cloud) {
    scan_data.curb_cloud.reset(new laserCloud());
    *(scan_data.curb_cloud) = *cloud_range_;
    scan_data.cloud_filter_flag = false;  // 先暂存点云，为操作滤波，节省耗时
  }
  std::fill(laser_scan_.begin(), laser_scan_.end(),
            occ_map_config_.laser_max_range + 1.0);
  v_scan_data_.push_back(scan_data);
}

bool OccMap::cloud2LaserScan(const laserCloud::Ptr cloud) {
  // laser_scan_.clear();
  // convert point to scan
  int cloudSize = cloud->points.size();
  if (0 == cloudSize) {
    return false;
  }

  cloud_range_->clear();

  // 路沿点的更新
  if (occ_map_config_.use_curb_cloud || occ_map_config_.use_view_cloud) {
    rangeFilter(cloud);
  }
  for (int i = 0; i < cloudSize; ++i) {
    PointType point = cloud->points[i];
    float x = point.x;
    float y = point.y;
    float range = std::sqrt(x * x + y * y);

    float angle = std::atan2(y, x);
    if (range > occ_map_config_.laser_max_range || point.z > 0.5) {
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

bool OccMap::rangeFilter(const laserCloud::Ptr cloud) {
  int cloud_size = cloud->points.size();
  for (int i = 0; i < cloud_size; ++i) {
    PointType point = cloud->points[i];
    float range = std::sqrt(point.x * point.x + point.y * point.y);
    if (range < occ_map_config_.curb_max_range &&
        range > occ_map_config_.curb_min_range &&
        point.z < occ_map_config_.view_max_height) {
      cloud_range_->push_back(point);
    }
  }
}

void OccMap::curbFilter(const laserCloud::Ptr cloud) {
  int cloud_size = cloud->points.size();
  for (int i = 0; i < occ_map_config_.curb_map_length; ++i) {
    for (int j = 0; j < occ_map_config_.curb_map_length; ++j) {
      init_flag_[i][j] = false;
    }
  }

  float local_map_origin_x = -occ_map_config_.curb_max_range;
  float local_map_orgin_y = -occ_map_config_.curb_max_range;
  for (int i = 0; i < cloud_size; ++i) {
    PointType point = cloud->points[i];
    float x = point.x;
    float y = point.y;
    float range = std::sqrt(x * x + y * y);
    if (range > occ_map_config_.curb_min_range &&
        range < occ_map_config_.curb_max_range) {
      int idx = (cloud->points[i].x - local_map_origin_x) /
                occ_map_config_.curb_map_resolution;
      int idy = (cloud->points[i].y - local_map_orgin_y) /
                occ_map_config_.curb_map_resolution;
      if (idx < 0 || idy < 0 || idx >= occ_map_config_.curb_map_length ||
          idy >= occ_map_config_.curb_map_length) {
        continue;
      }

      if (init_flag_[idx][idy] == false) {
        min_height_[idx][idy] = cloud->points[i].z;
        max_height_[idx][idy] = cloud->points[i].z;
        init_flag_[idx][idy] = true;
      } else {
        min_height_[idx][idy] =
            std::min(min_height_[idx][idy], cloud->points[i].z);
        max_height_[idx][idy] =
            std::max(max_height_[idx][idy], cloud->points[i].z);
      }
    }
  }

  // covert from height map to point cloud
  for (int i = 5; i < occ_map_config_.curb_map_length - 5; i = i + 10) {
    for (int j = 1; j < occ_map_config_.curb_map_length - 1; j = j + 2) {
      // no points at this grid
      bool is_curb = true;
      int count_p = 0;
      for (int k = i - 5; k < i + 5; k++) {
        for (int m = j - 1; m < j + 1; m++) {
          if (init_flag_[k][m] == true) {
            PointType this_point;
            this_point.x = local_map_origin_x +
                           k * occ_map_config_.curb_map_resolution +
                           occ_map_config_.curb_map_resolution / 2.0;
            this_point.y = local_map_orgin_y +
                           m * occ_map_config_.curb_map_resolution +
                           occ_map_config_.curb_map_resolution / 2.0;
            this_point.z = max_height_[k][m];

            if ((min_height_[k][m] < (occ_map_config_.ground_height + 0.1)) &&
                (min_height_[k][m] > (occ_map_config_.ground_height - 0.1)) &&
                (max_height_[k][m] < (occ_map_config_.ground_height + 0.2)) &&
                (this_point.y > occ_map_config_.curb_y_limit ||
                 this_point.y < (-1.0 * occ_map_config_.curb_y_limit))) {
              count_p++;
            }
          }
        }
      }
      if (count_p > 5) {
        for (int k = i - 5; k < i + 5; k++) {
          for (int m = j - 1; m < j + 1; m++) {
            if (init_flag_[k][m] == false) {
              continue;
            }
            PointType this_point;
            this_point.x = local_map_origin_x +
                           k * occ_map_config_.curb_map_resolution +
                           occ_map_config_.curb_map_resolution / 2.0;
            this_point.y = local_map_orgin_y +
                           m * occ_map_config_.curb_map_resolution +
                           occ_map_config_.curb_map_resolution / 2.0;
            this_point.z = max_height_[k][m];

            if ((min_height_[k][m] < (occ_map_config_.ground_height + 0.1)) &&
                (min_height_[k][m] > (occ_map_config_.ground_height - 0.1)) &&
                (max_height_[k][m] < (occ_map_config_.ground_height + 0.2)) &&
                (this_point.y > occ_map_config_.curb_y_limit ||
                 this_point.y < (-1.0 * occ_map_config_.curb_y_limit))) {
              cloud_curb_->push_back(this_point);
            }
          }
        }
      }
    }
  }
}

bool OccMap::updateGlobalMap() {
  const double &ang_min = occ_map_config_.laser_min_angle;
  // const double &ang_max = occ_map_config_.laser_max_angle;
  const double &ang_inc = occ_map_config_.laser_angle_increment;
  const double &range_max = occ_map_config_.laser_max_range;
  const double &range_min = occ_map_config_.laser_min_range;

  /* 设置遍历的步长，沿着一条激光线遍历 */
  const double &cell_size = getCellSize();
  const double &inc_step = 1.0 * cell_size;
  const unsigned int &scan_data_size = v_scan_data_.size();
  if (last_scan_data_size_ == scan_data_size) {
    return false;
  }
  if (v_scan_data_.empty()) {
    return false;
  }

  if (occ_map_config_.use_view_cloud || occ_map_config_.use_curb_cloud) {
    for (unsigned int i = last_scan_data_size_; i < scan_data_size; i++) {
      cloud_view_->clear();
      cloud_curb_->clear();
      if (!(v_scan_data_[i].cloud_filter_flag)) {
        if (occ_map_config_.use_view_cloud) {
          downSize_filter_.setInputCloud(v_scan_data_[i].curb_cloud);
          downSize_filter_.filter(*cloud_view_);
          v_scan_data_[i].view_cloud.reset(new laserCloud());
          *(v_scan_data_[i].view_cloud) = *cloud_view_;
        }
        if (occ_map_config_.use_curb_cloud) {
          curbFilter(v_scan_data_[i].curb_cloud);
          v_scan_data_[i].curb_cloud.reset(new laserCloud());
          *(v_scan_data_[i].curb_cloud) = *cloud_curb_;
        }
        v_scan_data_[i].cloud_filter_flag = true;
      }
    }
  }
  if (occ_map_config_.use_view_cloud) {
    for (unsigned int i = last_scan_data_size_; i < scan_data_size; i++) {
      Mat34d frame_pos;
      bool flag =
          ptr_map_manager_->getPoseFromID(v_scan_data_[i].frame_id, frame_pos);
      Mat34d base_frame_pose = Mathbox::multiplePose34d(frame_pos, T_lo_);
      if (flag) {
        Mat34d fixed_frame_pos = Mathbox::multiplePose34d(
            base_frame_pose, v_scan_data_[i].delta_pose);
        Pose2d base_pose = Mathbox::Mat34d2Pose2d(fixed_frame_pos);
        // 细节点的更新
        if (v_scan_data_[i].view_cloud != nullptr) {
          for (size_t k = 0; k < v_scan_data_[i].view_cloud->size(); k++) {
            float x = v_scan_data_[i].view_cloud->points[k].x;
            float y = v_scan_data_[i].view_cloud->points[k].y;
            Vec2d p_l(x, y);  //在激光雷达坐标系下的坐标
            Vec2d p_w = base_pose * p_l;
            updateCurb(p_w, occ_map_config_.view_value);
          }
        }
      }
    }
  }

  for (unsigned int i = last_scan_data_size_; i < scan_data_size; i++) {
    Mat34d frame_pos;
    bool flag =
        ptr_map_manager_->getPoseFromID(v_scan_data_[i].frame_id, frame_pos);
    Mat34d base_frame_pose = Mathbox::multiplePose34d(frame_pos, T_lo_);
    if (flag) {
      Mat34d fixed_frame_pos =
          Mathbox::multiplePose34d(base_frame_pose, v_scan_data_[i].delta_pose);
      Pose2d base_pose = Mathbox::Mat34d2Pose2d(fixed_frame_pos);
      Vec2d last_grid(Eigen::Infinity, Eigen::Infinity);

      /* for every laser beam */
      int count = 0;
      for (size_t j = 0; j < v_scan_data_[i].v_laser_scan.size(); j++) {
        /* 获取当前beam的距离 */
        const double &R = v_scan_data_[i].v_laser_scan[j];
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
          Vec2d p_w = base_pose * p_l;
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
    }
  }

  if (occ_map_config_.use_curb_cloud) {
    for (unsigned int i = last_scan_data_size_; i < scan_data_size; i++) {
      Mat34d frame_pos;
      bool flag =
          ptr_map_manager_->getPoseFromID(v_scan_data_[i].frame_id, frame_pos);
      Mat34d base_frame_pose = Mathbox::multiplePose34d(frame_pos, T_lo_);
      if (flag) {
        Mat34d fixed_frame_pos = Mathbox::multiplePose34d(
            base_frame_pose, v_scan_data_[i].delta_pose);
        Pose2d base_pose = Mathbox::Mat34d2Pose2d(fixed_frame_pos);
        // 细节点的更新
        if (v_scan_data_[i].curb_cloud != nullptr) {
          for (size_t k = 0; k < v_scan_data_[i].curb_cloud->size(); k++) {
            float x = v_scan_data_[i].curb_cloud->points[k].x;
            float y = v_scan_data_[i].curb_cloud->points[k].y;
            Vec2d p_l(x, y);  //在激光雷达坐标系下的坐标
            Vec2d p_w = base_pose * p_l;
            updateCurb(p_w, occ_map_config_.curb_value);
          }
        }
      }
    }
  }
  // increment update
  last_scan_data_size_ = v_scan_data_.size();

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

  global_occ_map_.info.width = map_width;
  global_occ_map_.info.height = map_height;
  global_occ_map_.info.resolution = mapResolution;
  global_occ_map_.data.clear();
  global_occ_map_.data.resize(map_width * map_height);
  std::fill(global_occ_map_.data.begin(), global_occ_map_.data.end(), -1);

  // local map origin x and y
  local_map_origin_point_.x = 0 - neg_max_cube_x * mapCubeLength;
  local_map_origin_point_.y = 0 - neg_max_cube_y * mapCubeLength;
  // local_map_origin_point_.z = origin_robot_point_.z;

  // update global mao origin
  global_occ_map_.info.origin(0, 3) = local_map_origin_point_.x;
  global_occ_map_.info.origin(1, 3) = local_map_origin_point_.y;

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
  local_map_origin_grid_.gridX =
      int((local_map_origin_point_.x - originCubeOriginX) / mapResolution);
  local_map_origin_grid_.gridY =
      int((local_map_origin_point_.y - originCubeOriginY) / mapResolution);

  for (int x = 0; x < map_width; x++) {
    for (int y = 0; y < map_height; y++) {
      int indX = local_map_origin_grid_.gridX + x;
      int indY = local_map_origin_grid_.gridY + y;

      Grid thisGrid;

      thisGrid.cubeX = local_map_origin_grid_.cubeX + indX / mapCubeArrayLength;
      thisGrid.cubeY = local_map_origin_grid_.cubeY + indY / mapCubeArrayLength;

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
        global_occ_map_.data[index] = (int8_t)(100 * thisCell->getBel());
      }
    }
  }
  return true;
}

void OccMap::updateCurb(const Vec2d &point, const double laser_inv_value) {
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
  thisCell->setCurbLogBel(laser_inv_value);  // 更新
}

void OccMap::updateChildMap(const Vec2d &point, const double laser_inv_value) {
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
  thisCell->setLogBel(laser_inv_value);  // 更新
}

bool OccMap::getPointGrid(const Vec2d &point, Grid &thisGrid) {
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
  thisGrid.gridX =
      (int) ((point[0] - map_array_[thisGrid.mapID].originX) / mapResolution);
  thisGrid.gridY =
      (int) ((point[1] - map_array_[thisGrid.mapID].originY) / mapResolution);
  if (thisGrid.gridX < 0 || thisGrid.gridY < 0 ||
      thisGrid.gridX >= mapCubeArrayLength ||
      thisGrid.gridY >= mapCubeArrayLength) {
    return false;
  }
  return true;
}

MapCell *OccMap::grid2MapCell(const Grid &grid) {
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

void OccMap::setMapFilePath(const std::string &path) {
  default_map_filepath_ = path;
}

void OccMap::initializeLocalOccupancyMap() {
  global_occ_map_.header.frame_id = "map";
  global_occ_map_.info.width = localMapArrayLength;
  global_occ_map_.info.height = localMapArrayLength;
  global_occ_map_.info.resolution = mapResolution;

  global_occ_map_.data.resize(global_occ_map_.info.width *
                              global_occ_map_.info.height);
}

void OccMap::saveMap(const std::string &map_dir) {
  LOG(WARNING) << "Save global occ map";
  clearTmpMap();
  updateGlobalMap();
  saveGlobalMapFile(global_occ_map_, map_dir);
  saveScan(map_dir);
  last_scan_data_size_ = 0;
  Reset();
  LOG(WARNING) << "Finish occ map";
}

bool OccMap::saveScan(const std::string &map_dir) {
  if (v_scan_data_.empty()) {
    return false;
  }
  std::ofstream scan_data_file;
  const std::string &scan_data_dir = map_dir + "scan_data.txt";
  scan_data_file.open(scan_data_dir);
  scan_data_file << std::fixed;
  if (!scan_data_file.is_open()) {
    LOG(ERROR) << " Can not open file";
    return false;
  }
  scan_data_file << "#format: frame_id tx ty tz qw qx qy qz point[....]"
                 << std::endl;
  size_t scan_index = 0;
  for (const auto &iter : v_scan_data_) {
    const size_t &frame_id = iter.frame_id;
    const Mat34d &delta_pose = iter.delta_pose;
    Eigen::Quaterniond quat(delta_pose.block<3, 3>(0, 0));
    quat.normalize();
    scan_data_file << std::setprecision(0) << frame_id << ' '
                   << std::setprecision(6) << delta_pose(0, 3) << ' '
                   << delta_pose(1, 3) << ' ' << delta_pose(2, 3) << ' '
                   << quat.x() << ' ' << quat.y() << ' ' << quat.z() << ' '
                   << quat.w();
    for (const auto &range_iter : iter.v_laser_scan) {
      scan_data_file << ' ' << range_iter;
    }
    scan_data_file << std::endl;

    if (occ_map_config_.use_curb_cloud && !(iter.curb_cloud)->empty()) {
      std::string corner_file =
          map_dir + "curb" + std::to_string(scan_index) + ".pcd";
      pcl::io::savePCDFileBinaryCompressed(corner_file, *(iter.curb_cloud));
    }
    if (occ_map_config_.use_view_cloud && !(iter.view_cloud)->empty()) {
      std::string corner_file =
          map_dir + "view" + std::to_string(scan_index) + ".pcd";
      pcl::io::savePCDFileBinaryCompressed(corner_file, *(iter.view_cloud));
    }
    scan_index++;
  }
  scan_data_file.close();
  return true;
}

bool OccMap::loadScan(const std::string &map_dir) {
  std::ifstream scan_data_file;
  const std::string &scan_data_dir = map_dir + "scan_data.txt";
  scan_data_file.open(scan_data_dir);
  if (!scan_data_file.is_open()) {
    LOG(ERROR) << " Can not open file";
    return false;
  }
  size_t scan_index = 0;
  while (!scan_data_file.eof()) {
    ScanData scan_data;
    double pose[7] = {0.};
    std::string s;
    std::getline(scan_data_file, s);
    if (s.front() == '#' || s.empty()) {
      continue;
    }
    std::stringstream ss;
    ss << s;
    ss >> scan_data.frame_id;

    for (uint i = 0; i < 7; ++i) { ss >> pose[i]; }
    Eigen::Quaterniond Quat(pose[6], pose[3], pose[4], pose[5]);
    Eigen::Map<Eigen::Vector3d> Trans(pose);
    Quat.normalize();
    scan_data.delta_pose.block<3, 3>(0, 0) = Quat.toRotationMatrix();
    scan_data.delta_pose.block<3, 1>(0, 3) = Trans;
    float range = 100;
    scan_data.v_laser_scan.reserve(range_size_);
    while (!ss.eof()) {
      ss >> range;
      scan_data.v_laser_scan.push_back(range);
    }

    if (occ_map_config_.use_curb_cloud) {
      scan_data.curb_cloud.reset(new laserCloud());
      std::string corner_file =
          map_dir + "curb" + std::to_string(scan_index) + ".pcd";
      if (pcl::io::loadPCDFile<PointType>(corner_file,
                                          *(scan_data.curb_cloud)) == -1) {
        LOG(INFO) << "curb cloud is empty" << scan_index;
      }
      scan_data.cloud_filter_flag = true;
    }

    if (occ_map_config_.use_view_cloud) {
      scan_data.view_cloud.reset(new laserCloud());
      std::string corner_file =
          map_dir + "view" + std::to_string(scan_index) + ".pcd";
      if (pcl::io::loadPCDFile<PointType>(corner_file,
                                          *(scan_data.view_cloud)) == -1) {
        LOG(INFO) << "view cloud is empty" << scan_index;
      }
      scan_data.cloud_filter_flag = true;
    }

    if (range_size_ == scan_data.v_laser_scan.size()) {
      v_scan_data_.emplace_back(scan_data);
    }
    scan_index++;
    // std::cout << std::setprecision(0) << scan_data.frame_id << ' '
    //           << std::setprecision(6) << pose[0] << ' ' << pose[1] << ' ' <<
    //           pose[2] << ' '
    //           << pose[3] << ' ' << pose[4] << ' ' << pose[5] << ' '<< pose[6]
    //           << ' '
    //           << scan_data.v_laser_scan.size() << ' ' <<
    //           scan_data.v_laser_scan.front() << ' '
    //           << scan_data.v_laser_scan.back() << std::endl;
  }
  LOG(WARNING) << "v_scan_data size after load " << v_scan_data_.size()
               << std::endl;
  return true;
}

double OccMap::laserInvModel(const double &r, const double &R,
                             const double &cell_size) {
  // double P_occ = 0.6;
  // double P_free = 0.4;
  // double P_prior = 0.5;
  if (r < (R - 0.5 * cell_size))
    return -0.3;  // P_free;

  if (r > (R + 0.5 * cell_size))
    return 0;  //对数概率　P_prior;

  return 0.2;  // P_occ;
}

void OccMap::saveGlobalMap() {
  saveGlobalMapFile(global_occ_map_, default_map_filepath_);
}

void OccMap::saveGlobalMapFile(const UserOccupancyGrid &map,
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
