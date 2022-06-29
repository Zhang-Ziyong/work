#include "occupancy_map/depth_camera_occ_map.hpp"
#include <sys/file.h>
#include <fstream>
#include <string>
#include <functional>
namespace cvte_lidar_slam {
std::shared_ptr<DepthOccupancyMap> DepthOccupancyMap::ptr_occ_map_ = nullptr;

DepthOccupancyMap::DepthOccupancyMap() {
  default_map_filepath_ = "./";

  map_type_ = "scan";
  mapping_mode_ = false;

  allocateMemory();

  map_array_count_ = 0;

  LOG(INFO) << "construct new object <DepthOccupancyMap> successfully";
}

std::shared_ptr<DepthOccupancyMap> DepthOccupancyMap::getInstance() {
  if (ptr_occ_map_ == nullptr) {
    ptr_occ_map_.reset(new DepthOccupancyMap());
  }
  return ptr_occ_map_;
}

void DepthOccupancyMap::allocateMemory() {
  // initialize array for cmap
  map_array_ind_ = new int *[mapArrayLength_];
  for (int i = 0; i < mapArrayLength_; ++i)
    map_array_ind_[i] = new int[mapArrayLength_];

  for (int i = 0; i < mapArrayLength_; ++i)
    for (int j = 0; j < mapArrayLength_; ++j) map_array_ind_[i][j] = -1;
}

void DepthOccupancyMap::insertObstacleCloud(
    const depthCloud::Ptr obstacle_cloud, const Mat34d &cur_pose,
    const size_t frame_id, const Mat34d &near_pose) {
  has_robot_point_ = true;

  Mat34d delta_pose = Mathbox::deltaPose34d(near_pose, cur_pose);

  if (is_first_depth_frame_) {
    last_frame_pose_ = cur_pose;
  } else {
    Mat34d delta_trans = Mathbox::deltaPose34d(last_frame_pose_, cur_pose);
    double delta_rot =
        Mathbox::rotationMatrixToEulerAngles(delta_trans.block<3, 3>(0, 0))
            .norm();
    double delta_translation = delta_trans.block<3, 1>(0, 3).norm();

    if (delta_rot < 0.02 && delta_translation < 0.05) {
      // LOG(WARNING) << "samll delta_translation and rot";
      return;
    }
  }
  last_frame_pose_ = cur_pose;
  is_first_depth_frame_ = false;
  /* 获取激光的信息 */
  ScanData scan_data;
  scan_data.delta_pose = delta_pose;
  scan_data.frame_id = frame_id;

  if (!cloud2LaserScan(obstacle_cloud)) {
    // LOG(WARNING) << "Empty obstacle_cloud";
    return;
  }

  if (laser_scan_.empty()) {
    // LOG(WARNING) << "Empty laser scan";
    return;
  }
  scan_data.v_laser_scan.reserve(laser_scan_.size());
  scan_data.v_laser_scan = laser_scan_;
  if (depth_occupy_map_config_.use_view_cloud) {
    scan_data.view_cloud.reset(new depthCloud());
    *(scan_data.view_cloud) = *obstacle_cloud;
  }

  std::fill(laser_scan_.begin(), laser_scan_.end(),
            depth_occupy_map_config_.laser_max_range + 1.0);
  {
    std::lock_guard<std::mutex> guard(v_scan_data_mutex_);
    v_scan_data_.push_back(scan_data);
  }
}

bool DepthOccupancyMap::cloud2LaserScan(const depthCloud::Ptr cloud) {
  // laser_scan_.clear();
  // convert point to scan
  int cloudSize = cloud->points.size();
  if (0 == cloudSize) {
    return false;
  }

  for (int i = 0; i < cloudSize; ++i) {
    depthPointType point = cloud->points[i];
    float x = point.x;
    float y = point.y;
    float range = std::sqrt(x * x + y * y);
    float angle = std::atan2(y, x);
    if (angle < depth_occupy_map_config_.laser_min_angle ||
        depth_occupy_map_config_.laser_max_angle < angle) {
      continue;
    }
    if (range > depth_occupy_map_config_.laser_max_range) {
      continue;
    }
    uint index = std::round((angle - depth_occupy_map_config_.laser_min_angle) /
                            depth_occupy_map_config_.laser_angle_increment);
    if (index < range_size_) {
      laser_scan_[index] = std::min(laser_scan_[index], range);
    }
  }
  return true;
}

bool DepthOccupancyMap::updateGlobalMap(
    const std::map<size_t, std::shared_ptr<KeyFrame>> &node_poses,
    const UserOccupancyGrid &global_map) {
  {
    std::lock_guard<std::mutex> guard(v_scan_data_mutex_);
    v_scan_data_tmp_ = v_scan_data_;
  }
  // clearTmpMap();

  mapResolution_ = global_map.info.resolution;
  mapCubeArrayLength_ = mapCubeLength_ / mapResolution_;
  localMapLength_ = 20;  // length of the local occupancy grid map (meter)
  localMapArrayLength_ = localMapLength_ / mapResolution_;

  if (!mapping_mode_) {
    return false;
  }

  const unsigned int &scan_data_size = v_scan_data_tmp_.size();
  // std::cout << "v_scan_data_ " << v_scan_data_.size() << ","
  //           << last_scan_data_size_ << std::endl;
  if (v_scan_data_tmp_.empty()) {
    global_occ_map_ = global_map;
    return true;
  }

  if (depth_occupy_map_config_.use_view_cloud) {
    unsigned int i = last_scan_data_size_;
    for (; i < scan_data_size; i++) {
      auto frame_iter = node_poses.find(v_scan_data_tmp_[i].frame_id);
      bool flag;
      if (frame_iter != node_poses.end()) {
        flag = true;
      } else {
        flag = false;
      }

      Pose2d curr_bot_pose;
      if (flag) {
        Mat34d frame_pos;
        frame_pos = (frame_iter->second)->getPose();
        Mat34d base_frame_pose = Mathbox::multiplePose34d(frame_pos, T_lo_);

        Mat34d fixed_frame_pos = Mathbox::multiplePose34d(
            base_frame_pose, v_scan_data_tmp_[i].delta_pose);
        Pose2d base_pose = Mathbox::Mat34d2Pose2d(fixed_frame_pos);
        curr_bot_pose = base_pose;

        // 获取梯形角点
        Vec2d left_down, left_up, right_down, right_up;
        left_down[0] = depth_occupy_map_config_.search_area_point[0];
        left_down[1] = depth_occupy_map_config_.search_area_point[1];
        left_up[0] = depth_occupy_map_config_.search_area_point[2];
        left_up[1] = depth_occupy_map_config_.search_area_point[3];
        right_down[0] = depth_occupy_map_config_.search_area_point[4];
        right_down[1] = depth_occupy_map_config_.search_area_point[5];
        right_up[0] = depth_occupy_map_config_.search_area_point[6];
        right_up[1] = depth_occupy_map_config_.search_area_point[7];
        left_down = curr_bot_pose * left_down;
        left_up = curr_bot_pose * left_up;
        right_down = curr_bot_pose * right_down;
        right_up = curr_bot_pose * right_up;

        if (v_scan_data_tmp_[i].view_cloud != nullptr) {
          for (size_t k = 0; k < v_scan_data_tmp_[i].view_cloud->size(); k++) {
            float x = v_scan_data_tmp_[i].view_cloud->points[k].x;
            float y = v_scan_data_tmp_[i].view_cloud->points[k].y;
            Vec2d p_l(x, y);  // 在激光雷达坐标系下的坐标
            Vec2d p_w = base_pose * p_l;

            bool judge_in_bool =
                judgeInArea(left_down, left_up, right_down, right_up, p_w);
            if (!judge_in_bool)
              continue;

            Grid thisGrid;
            if (!getPointGrid(p_w, thisGrid)) {
              continue;
            }
            // Get current cell pointer
            MapCell *thisCell = grid2MapCell(thisGrid);
            if (nullptr == thisCell) {
              continue;
            }

            // 进行概率更新
            // update occupancy
            thisCell->setCurbLogBel(
                depth_occupy_map_config_.add_bel_value,
                depth_occupy_map_config_.max_bel_value);  // 更新

            // 将thisGrid 添加到保存map 中
            std::string hash_string = std::to_string(thisGrid.cubeX) +
                                      std::to_string(thisGrid.cubeY) +
                                      std::to_string(thisGrid.gridX) +
                                      std::to_string(thisGrid.gridY);
            std::hash<std::string> hash_fn;
            size_t hash_string_value = hash_fn(hash_string);

            if (save_grid.find(hash_string_value) == save_grid.end()) {
              save_grid[hash_string_value] = thisGrid;
            }

            if (save_curr_scan_grid.find(hash_string_value) ==
                save_curr_scan_grid.end()) {
              save_curr_scan_grid[hash_string_value] = thisGrid;
            }
          }

          // 概率消除
          if (0 != i) {
            for (const auto &save_grid_iter : save_grid) {
              // Get current cell pointer
              MapCell *judge_cell = grid2MapCell(save_grid_iter.second);
              if (nullptr == judge_cell) {
                continue;
              }

              Vec2d currPoint;
              currPoint[0] = judge_cell->pointX;
              currPoint[1] = judge_cell->pointY;

              // 判断是否位于深度相机扫描区域
              bool judge_in_bool = judgeInArea(left_down, left_up, right_down,
                                               right_up, currPoint);
              if (judge_in_bool) {
                if (save_curr_scan_grid.find(save_grid_iter.first) ==
                    save_curr_scan_grid.end()) {
                  judge_cell->setCurbDownLogBel(
                      depth_occupy_map_config_.reduce_bel_value);
                }
              }
            }
          }
          save_curr_scan_grid.erase(save_curr_scan_grid.begin(),
                                    save_curr_scan_grid.end());
        }
      }
    }
  }

  // outDepthPoint.close();
  // increment update
  last_scan_data_size_ = v_scan_data_tmp_.size();
  global_occ_map_ = global_map;
  int map_width = global_occ_map_.info.width;
  int map_height = global_occ_map_.info.height;

  if (map_width <= 0 || map_height <= 0) {
    return false;
  }

  // 获取地图起始点
  local_map_origin_point_.x = global_occ_map_.info.origin(0, 3);
  local_map_origin_point_.y = global_occ_map_.info.origin(1, 3);

  // local map origin cube id (in global map)
  local_map_origin_grid_.cubeX =
      static_cast<int>((local_map_origin_point_.x + mapCubeLength_ / 2.0) /
                       mapCubeLength_) +
      rootCubeIndex_;
  local_map_origin_grid_.cubeY =
      static_cast<int>((local_map_origin_point_.y + mapCubeLength_ / 2.0) /
                       mapCubeLength_) +
      rootCubeIndex_;
  if (local_map_origin_point_.x + mapCubeLength_ / 2.0 < 0)
    --local_map_origin_grid_.cubeX;
  if (local_map_origin_point_.y + mapCubeLength_ / 2.0 < 0)
    --local_map_origin_grid_.cubeY;
  // local map origin grid id (in sub-map)
  float originCubeOriginX,
      originCubeOriginY;  // the orign of submap that the local map origin
  // belongs to (note the submap may not be created yet,
  // cannot use originX and originY)
  originCubeOriginX =
      (local_map_origin_grid_.cubeX - rootCubeIndex_) * mapCubeLength_ -
      mapCubeLength_ / 2.0;
  originCubeOriginY =
      (local_map_origin_grid_.cubeY - rootCubeIndex_) * mapCubeLength_ -
      mapCubeLength_ / 2.0;
  local_map_origin_grid_.gridX = static_cast<int>(
      (local_map_origin_point_.x - originCubeOriginX) / mapResolution_);
  local_map_origin_grid_.gridY = static_cast<int>(
      (local_map_origin_point_.y - originCubeOriginY) / mapResolution_);

  // 开始更新地图
  for (int x = 0; x < map_width; x++) {
    for (int y = 0; y < map_height; y++) {
      int indX = local_map_origin_grid_.gridX + x;
      int indY = local_map_origin_grid_.gridY + y;

      Grid thisGrid;
      thisGrid.cubeX =
          local_map_origin_grid_.cubeX + indX / mapCubeArrayLength_;
      thisGrid.cubeY =
          local_map_origin_grid_.cubeY + indY / mapCubeArrayLength_;

      thisGrid.gridX = indX % mapCubeArrayLength_;
      thisGrid.gridY = indY % mapCubeArrayLength_;

      thisGrid.mapID = map_array_ind_[thisGrid.cubeX][thisGrid.cubeY];

      // if sub-map is not created yet
      if (!subMapExist(thisGrid.cubeX, thisGrid.cubeY)) {
        continue;
      }

      MapCell *thisCell = grid2MapCell(thisGrid);
      // skip unknown grid
      if (nullptr != thisCell && fabs(thisCell->log_odds) > 0.01) {
        int index = x + y * map_width;  // index of the 1-D array
        if (thisCell->getBel() > 0.55) {
          global_occ_map_.data[index] =
              static_cast<int8_t>(100 * thisCell->getBel());
        }
      }
    }
  }

  return true;
}

bool DepthOccupancyMap::updateGlobalMap() {
  return true;
}

void DepthOccupancyMap::updateChildMap(const Vec2d &point,
                                       const double &laser_inv_value) {
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

void DepthOccupancyMap::startMapping(const std::string &path) {
  mapping_mode_ = true;
  default_map_filepath_ = path;
}

void DepthOccupancyMap::updateCurb(const Vec2d &point,
                                   const double &laser_inv_value) {
  (void) laser_inv_value;
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
  // thisCell->setCurbLogBel(laser_inv_value);  // 更新
}

void DepthOccupancyMap::initializeLocalOccupancyMap() {
  global_occ_map_.header.frame_id = "map";
  global_occ_map_.info.width = localMapArrayLength_;
  global_occ_map_.info.height = localMapArrayLength_;
  global_occ_map_.info.resolution = mapResolution_;

  global_occ_map_.data.resize(global_occ_map_.info.width *
                              global_occ_map_.info.height);
}

void DepthOccupancyMap::saveMap(const std::string &map_dir) {
  LOG(WARNING) << "Save global occ map";
  clearTmpMap();
  updateGlobalMap();
  saveGlobalMapFile(global_occ_map_, map_dir);
  saveScan(map_dir);
  last_scan_data_size_ = 0;
  Reset();
  LOG(WARNING) << "Finish occ map";
}

bool DepthOccupancyMap::saveScan(const std::string &map_dir) {
  if (v_scan_data_.empty()) {
    return false;
  }
  std::ofstream scan_data_file;
  const std::string &scan_data_dir = map_dir + "map_2d_scan_data.txt";
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

    if (depth_occupy_map_config_.use_view_cloud &&
        !(iter.view_cloud)->empty()) {
      std::string corner_file =
          map_dir + "map_2d_obs" + std::to_string(scan_index) + ".pcd";
      pcl::io::savePCDFileBinaryCompressed(corner_file, *(iter.view_cloud));
    }
    scan_index++;
  }
  scan_data_file.close();
  return true;
}

bool DepthOccupancyMap::loadScan(const std::string &map_dir) {
  std::ifstream scan_data_file;
  const std::string &scan_data_dir = map_dir + "map_2d_scan_data.txt";
  scan_data_file.open(scan_data_dir);
  if (!scan_data_file.is_open()) {
    LOG(ERROR) << " Can not open file " << scan_data_dir;
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
    if (depth_occupy_map_config_.use_view_cloud) {
      scan_data.view_cloud.reset(new depthCloud());
      std::string corner_file =
          map_dir + "map_2d_obs" + std::to_string(scan_index) + ".pcd";
      if (pcl::io::loadPCDFile<depthPointType>(corner_file,
                                               *(scan_data.view_cloud)) == -1) {
        // LOG(INFO) << "view cloud is empty" << scan_index;
      }
    }
    if (range_size_ == scan_data.v_laser_scan.size()) {
      v_scan_data_.emplace_back(scan_data);
    }
    scan_index++;
  }
  LOG(WARNING) << "v_scan_data size after load " << v_scan_data_.size()
               << std::endl;
  return true;
}

double DepthOccupancyMap::laserInvModel(const double &r, const double &R,
                                        const double &cell_size) {
  if (r < (R - 0.5 * cell_size))
    return -0.3;  // P_free;

  if (r > (R + 0.5 * cell_size))
    return 0;  // 对数概率　P_prior;

  return 0.4;  // P_occ;
}

void DepthOccupancyMap::saveGlobalMap() {
  saveGlobalMapFile(global_occ_map_, default_map_filepath_);
}

void DepthOccupancyMap::saveGlobalMapYaml() {
  saveGlobalMapYamlFile(global_occ_map_, default_map_filepath_);
}

UserOccupancyGrid DepthOccupancyMap::getCurrOccMap() {
  return global_occ_map_;
}
}  // namespace cvte_lidar_slam
