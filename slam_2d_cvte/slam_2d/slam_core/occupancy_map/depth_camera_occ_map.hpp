/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 201９, CVTE.
 * All rights reserved.
 *
 *@file base_map.hpp
 *
 *@brief
 * 1.深度相机占用栅格地图生成
 *
 *@author caoyong(caoyong@cvte.com)
 *@version V1.0
 *@data 2021-04-02
 ************************************************************************/

#ifndef _SLAM_2D_DEPTH_CAMERA_OCC_MAP
#define _SLAM_2D_DEPTH_CAMERA_OCC_MAP

#include <glog/logging.h>
#include <sys/file.h>
#include <atomic>
#include <mutex>

#include "common/data_struct/keyframe.hpp"
#include "common/config/system_config.hpp"
#include "occupancy_map/depth_camera_calibrate.hpp"
#include "occupancy_map/depth_camera_option.hpp"
#include "occupancy_map/occupancy_map_common.hpp"

namespace cvte_lidar_slam {
class DepthOccupancyMap {
 public:
  ~DepthOccupancyMap() {
    map_array_.clear();
    std::vector<ChildMap>().swap(map_array_);
    save_grid.erase(save_grid.begin(), save_grid.end());
  }
  static std::shared_ptr<DepthOccupancyMap> getInstance();

  void updateParameter();

  void initLaserScanParam();

  void insertObstacleCloud(const depthCloud::Ptr obstacle_cloud,
                           const Mat34d &near_pose, const size_t frame_id,
                           const Mat34d &cur_pose);

  bool useOcc() { return calib_config_.use_depth_camera; }

  void setCameraCalibrateOption(const DepthCameraOptions &calib_config) {
    calib_config_ = calib_config;
    ptr_camera_calibrate_ =
        std::make_shared<DepthCameraCalibrate>(calib_config_);
    ptr_camera_calibrate_->setTF(calib_config);
  }

  void setDepthOccMapOption(
      const DepthOccupancyMapOptions &depth_occupy_map_config) {
    depth_occupy_map_config_ = depth_occupy_map_config;
    // std::cout << "depth_occupy_map_config_"
    //           << depth_occupy_map_config_.search_area_point[0] << ","
    //           << depth_occupy_map_config_.search_area_point[1] << std::endl;
    range_size_ = std::ceil((depth_occupy_map_config_.laser_max_angle -
                             depth_occupy_map_config_.laser_min_angle) /
                            depth_occupy_map_config_.laser_angle_increment);
    laser_scan_.resize(range_size_,
                       depth_occupy_map_config_.laser_max_range + 1.);
  }

  void transformCloud(const depthCloud::Ptr &cloud_in,
                      depthCloud::Ptr &cloud_out, const int &flag) {
    if (nullptr == ptr_camera_calibrate_) {
      return;
    }
    depthCloud::Ptr ground_cloud(new depthCloud());
    depthCloud::Ptr laser_cloud(new depthCloud());
    depthCloud::Ptr target_cloud(new depthCloud());

    Eigen::Matrix4f trans_matrix4f_ = Eigen::Matrix4f::Identity();

    if (1 == flag) {
      trans_matrix4f_ = ptr_camera_calibrate_->frontUpGetTransform();
    } else {
      trans_matrix4f_ = ptr_camera_calibrate_->frontDownGetTransform();
    }

    pcl::transformPointCloud(*cloud_in, *target_cloud, trans_matrix4f_);

    const size_t &number = target_cloud->size();
    laser_cloud->points.reserve(1000);

    for (size_t i = 0; i < number; ++i) {
      const auto pi = target_cloud->points[i];
      const double dis = sqrt(pi.x * pi.x + pi.y * pi.y + pi.z * pi.z);
      if (dis > ptr_camera_calibrate_->depth_dist_value) {
        continue;
      }

      if (calib_config_.high_filter_value > pi.z) {
        continue;
      }

      laser_cloud->points.push_back(pi);
    }
    cloud_out = laser_cloud;
  }

  bool updateGlobalMap();

  bool updateGlobalMap(
      const std::map<size_t, std::shared_ptr<KeyFrame>> &node_poses,
      const UserOccupancyGrid &map);

  void saveGlobalMapYaml();
  void saveGlobalMapYamlFile(const UserOccupancyGrid &map,
                             const std::string &map_dir) {
    LOG(ERROR) << "Received a " << map.info.width << "X" << map.info.height
               << "map " << map.info.resolution << "m/pix";

    std::string mapmetadatafile = map_dir + ".yaml";
    LOG(ERROR) << "Writing map occupancy data yaml to " << mapmetadatafile;
    FILE *yaml = fopen(mapmetadatafile.c_str(), "w");
    flock(yaml->_fileno, LOCK_EX);

    Pose2d laser_pose = Mathbox::Mat34d2Pose2d(map.info.origin);

    fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\n",
            mapmetadatafile.c_str(), map.info.resolution, laser_pose.x(),
            laser_pose.y(), laser_pose.yaw());
    fprintf(yaml, "height: %d\nwidth: %d\n", map.info.height, map.info.width);
    fprintf(yaml, "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n");
    LOG(ERROR) << "22Writing map occupancy data yaml to " << mapmetadatafile;
    flock(yaml->_fileno, LOCK_UN);
    fclose(yaml);
    LOG(ERROR) << "Received a 2" << map.info.width << "X" << map.info.height
               << "map " << map.info.resolution << "m/pix";
  }

  void saveGlobalMap();
  // TODO(zhangziyong): 去掉
  void saveGlobalMapFile(const UserOccupancyGrid &map,
                         const std::string &map_dir) {
    LOG(INFO) << "Received a " << map.info.width << "X" << map.info.height
              << "map " << map.info.resolution << "m/pix";
    const int threshold_free = 49;
    const int threshold_occupied = 55;
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

    std::string map_name;
    auto pos = mapdatafile.find_last_of("/");
    if (pos == std::string::npos) {
      map_name = mapdatafile;
    } else if (pos != (mapdatafile.size() - 1)) {
      map_name = mapdatafile.substr(pos + 1);
    } else {
      LOG(ERROR) << "map name error!";
      return;
    }

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

  void setGlobalMap(const UserOccupancyGrid &map) { global_occ_map_ = map; }
  UserOccupancyGrid getGlobalMap() { return global_occ_map_; }

  bool loadScan(const std::string &map_dir);
  bool saveScan(const std::string &map_dir);

  void Reset() {
    v_scan_data_.clear();
    std::vector<ScanData>().swap(v_scan_data_);
    v_scan_data_.reserve(200);
    map_array_.clear();
    save_grid.erase(save_grid.begin(), save_grid.end());
    std::vector<ChildMap>().swap(map_array_);
    map_array_.reserve(100);
    map_array_count_ = 0;
    last_scan_data_size_ = 0;
    is_first_depth_frame_ = true;
    is_stop_requested_ = false;
    mapping_mode_ = false;
    for (int i = 0; i < mapArrayLength_; ++i) {
      for (int j = 0; j < mapArrayLength_; ++j) { map_array_ind_[i][j] = -1; }
    }
  }

  void saveMap(const std::string &map_dir);
  void startMapping(const std::string &path);

  void setConfig(const OccMapConfig &config) {}

  bool setExtrincs(const Mat34d &T_lo) {
    if (!T_lo.hasNaN()) {
      T_lo_ = T_lo;
      LOG(WARNING) << "setExtrincs T_lo_:\n" << T_lo_;
      return true;
    }
    return false;
  }

  void clearTmpMap() {
    map_array_.clear();
    save_grid.erase(save_grid.begin(), save_grid.end());
    std::vector<ChildMap>().swap(map_array_);
    map_array_.reserve(100);
    map_array_count_ = 0;
    last_scan_data_size_ = 0;
    for (int i = 0; i < mapArrayLength_; ++i) {
      for (int j = 0; j < mapArrayLength_; ++j) { map_array_ind_[i][j] = -1; }
    }
  }

  void clearTmpMapWithLock() {
    map_array_.clear();
    save_grid.erase(save_grid.begin(), save_grid.end());
    std::vector<ChildMap>().swap(map_array_);
    map_array_.reserve(100);
    map_array_count_ = 0;
    last_scan_data_size_ = 0;
    for (int i = 0; i < mapArrayLength_; ++i) {
      for (int j = 0; j < mapArrayLength_; ++j) { map_array_ind_[i][j] = -1; }
    }
  }

  std::string getOccMapType() { return map_type_; }
  void setTbl(const Mat34d t_bl) { T_bl_ = t_bl; }

  UserOccupancyGrid getCurrOccMap();
  void inline resetUpdateValue() { clearTmpMap(); };

 private:
  bool getPointGrid(const Vec2d &point, Grid &thisGrid) {
    thisGrid.cubeX =
        static_cast<int>((point[0] + mapCubeLength_ / 2.0) / mapCubeLength_) +
        rootCubeIndex_;
    thisGrid.cubeY =
        static_cast<int>((point[1] + mapCubeLength_ / 2.0) / mapCubeLength_) +
        rootCubeIndex_;
    if (point[0] + mapCubeLength_ / 2.0 < 0.) {
      --thisGrid.cubeX;
    }
    if (point[1] + mapCubeLength_ / 2.0 < 0.) {
      --thisGrid.cubeY;
    }

    // Decide whether a point is out of pre-allocated map
    if (thisGrid.cubeX >= 0 && thisGrid.cubeX < mapArrayLength_ &&
        thisGrid.cubeY >= 0 && thisGrid.cubeY < mapArrayLength_) {
      // Point is in the boundary, but this sub-map is not allocated before
      // Allocate new memory for this sub-map and save it to mapArray
      if (-1 == map_array_ind_[thisGrid.cubeX][thisGrid.cubeY]) {
        map_array_.emplace_back(map_array_count_, thisGrid.cubeX,
                                thisGrid.cubeY, rootCubeIndex_, mapCubeLength_,
                                mapCubeArrayLength_);
        map_array_ind_[thisGrid.cubeX][thisGrid.cubeY] = map_array_count_;
        ++map_array_count_;
      }
    } else {
      LOG(ERROR)
          << "Point cloud is out of elevation map boundary. Change params "
             "->mapArrayLength<-. The program will crash!";
      return false;
    }
    // sub-map id
    thisGrid.mapID = map_array_ind_[thisGrid.cubeX][thisGrid.cubeY];
    // Find the index for this point in this sub-map (grid index)
    thisGrid.gridX = static_cast<int>(
        (point[0] - map_array_[thisGrid.mapID].originX) / mapResolution_);
    thisGrid.gridY = static_cast<int>(
        (point[1] - map_array_[thisGrid.mapID].originY) / mapResolution_);
    if (thisGrid.gridX < 0 || thisGrid.gridY < 0 ||
        thisGrid.gridX >= mapCubeArrayLength_ ||
        thisGrid.gridY >= mapCubeArrayLength_) {
      return false;
    }
    thisGrid.pointX = point[0];
    thisGrid.pointY = point[1];
    return true;
  }

  MapCell *grid2MapCell(const Grid &grid) {
    MapCell *p_cell = nullptr;
    try {
      p_cell =
          &(map_array_.at(grid.mapID).cellArray.at(grid.gridX).at(grid.gridY));

    } catch (...) {
      LOG(ERROR) << grid.mapID << " " << grid.gridX << " " << grid.gridY
                 << std::endl;
    }
    p_cell->pointX = grid.pointX;
    p_cell->pointY = grid.pointY;
    return p_cell;
  }

  double pointDistance(const Vec2d &point_1, const Vec2d &point_2) {
    double distance = 0;
    distance = std::sqrt((point_1[0] - point_2[0]) * (point_1[0] - point_2[0]) +
                         (point_1[1] - point_2[1]) * (point_1[1] - point_2[1]));
    return distance;
  }

  double triangleArea(const Vec2d &point_1, const Vec2d &point_2,
                      const Vec2d &point_3) {
    double area = 0;
    double a = 0, b = 0, c = 0, s = 0;
    a = pointDistance(point_1, point_2);
    b = pointDistance(point_1, point_3);
    c = pointDistance(point_2, point_3);
    s = 0.5 * (a + b + c);
    area = std::sqrt(s * (s - a) * (s - b) * (s - c));
    return area;
  }

  bool judgeInArea(const Vec2d &left_down, const Vec2d &left_up,
                   const Vec2d &right_down, const Vec2d &right_up,
                   const Vec2d &curr_point) {
    double triangle_area_01, triangle_area_02, triangle_area_03,
        triangle_area_04, triangle_area_05, triangle_area_06;

    // TODO:共线会不会有bug
    triangle_area_01 = triangleArea(left_down, left_up, curr_point);
    triangle_area_02 = triangleArea(left_up, right_up, curr_point);
    triangle_area_03 = triangleArea(right_up, right_down, curr_point);
    triangle_area_04 = triangleArea(right_down, left_down, curr_point);

    triangle_area_05 = triangleArea(left_down, left_up, right_up);
    triangle_area_06 = triangleArea(right_up, right_down, left_down);

    double judeg_value =
        std::abs(triangle_area_01 + triangle_area_02 + triangle_area_03 +
                 triangle_area_04 - triangle_area_05 - triangle_area_06);

    if (judeg_value < 0.0001)
      return true;

    return false;
  }

  /**
   *subMapExist
   *@brief
   *判断子图是否存在
   *
   *@param[in] cube_x-x坐标
   *@param[in] cube_y-y坐标
   **/
  bool subMapExist(const int cube_x, const int cube_y) {
    if (cube_x >= mapArrayLength_ || cube_y >= mapArrayLength_) {
      return false;
    }
    if (-1 == map_array_ind_[cube_x][cube_y]) {
      return false;
    } else {
      return true;
    }
  }

  unsigned int getMapArraySize() { return map_array_.size(); }
  int getMapArrayIndX(const int index) { return map_array_[index].indX; }
  int getMapArrayIndY(const int index) { return map_array_[index].indY; }
  float getCellSize() { return mapResolution_; }

  float pointDistance(PointType p1, PointType p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
                (p1.z - p2.z) * (p1.z - p2.z));
  }

 private:
  void allocateMemory();

  void initializeLocalOccupancyMap();

  bool cloud2LaserScan(const depthCloud::Ptr cloud);

  void updateChildMap(const Vec2d &point, const double &laser_inv_value);
  void updateCurb(const Vec2d &point, const double &laser_inv_value);

  double laserInvModel(const double &r, const double &R,
                       const double &cell_size);

  bool getRobotPosition();

  void getMap();

  void getLocalMap();

  void initOccupancyMap();

 private:
  int **
      map_array_ind_;  ///< it saves the index of this submap in vector mapArray

  // Map Arrays
  int map_array_count_;
  std::vector<ChildMap> map_array_;
  std::string map_type_;
  Mat34d T_bl_ = Mathbox::Identity34();

  std::shared_ptr<DepthCameraCalibrate> ptr_camera_calibrate_;

  DepthCameraOptions calib_config_;
  DepthOccupancyMapOptions depth_occupy_map_config_;

  Mat34d last_frame_pose_;
  bool is_first_depth_frame_ = true;

  UserOccupancyGrid global_occ_map_;

  // 模块数据
  Grid local_map_origin_grid_;
  Grid global_map_origin_grid_;

  std::vector<ScanData> v_scan_data_;
  std::vector<ScanData> v_scan_data_tmp_;
  std::vector<float> laser_scan_;
  u_int32_t range_size_;

  size_t last_scan_data_size_ = 0;
  std::atomic<bool> is_stop_requested_;
  int pub_count_;

  PointType robot_point_;
  PointType local_map_origin_point_;
  PointType origin_robot_point_;
  PointType global_map_origin_point_;

  Pose2d g_robot_pose_;
  Mat34d robot_pose_;

  std::mutex robot_transform_mtx_;
  std::atomic<bool> has_robot_point_;
  std::string default_map_filepath_;
  std::atomic<bool> mapping_mode_;

  std::map<size_t, Grid> save_grid;
  std::map<size_t, Grid> save_curr_scan_grid;

  float mapCubeLength_ = 1.0;  // the length of a sub-map (meters)
  float mapResolution_ = 0.05;
  int mapCubeArrayLength_ = mapCubeLength_ / mapResolution_;
  int mapArrayLength_ = 2000 / mapCubeLength_;
  int rootCubeIndex_ = mapArrayLength_ / 2;

  // 2D Map Publish Params
  int localMapLength_ = 20;  // length of the local occupancy grid map (meter)
  int localMapArrayLength_ = localMapLength_ / mapResolution_;

  // empty depth
  int empth_depth_count_ = 0;

  Mat34d T_lo_;

  //
  std::mutex v_scan_data_mutex_;

 private:
  DepthOccupancyMap();
  static std::shared_ptr<DepthOccupancyMap> ptr_occ_map_;
};
}  // namespace cvte_lidar_slam

#endif
