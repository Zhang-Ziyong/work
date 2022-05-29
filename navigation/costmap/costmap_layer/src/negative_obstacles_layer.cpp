#include "negative_obstacles_layer.hpp"

#include "costmap_mediator.hpp"
namespace CVTE_BABOT {
bool NegativeObstaclesLayer::onInitialize() {
  if (!ObstacleLayer::onInitialize()) {
    LOG(ERROR) << "onInitialize failed";
    return false;
  }
  LOG(INFO) << "init NegativeObstaclesLayer";
  return true;
}

void NegativeObstaclesLayer::getParams() {
  ObstacleLayer::getParams();
  if (!b_enabled_) {
    return;
  }
}

bool NegativeObstaclesLayer::updateBounds(const WorldmapPose &wp_robot_pose,
                                          CostmapBound &cb_costmap_bound) {
  LOG(INFO) << " NegativeObstaclesLayer updateBounds";
  if (!b_enabled_) {
    LOG(INFO) << "NegativeObstaclesLayer not enable ";
    return false;
  }
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  if (b_rolling_window_) {
    updateOrigin(wp_robot_pose.d_x - getSizeInMetersX() / 2,
                 wp_robot_pose.d_y - getSizeInMetersY() / 2);
  }

  useExtraBounds(cb_costmap_bound);
  if (getLayerUpdate()) {
    std::vector<std::shared_ptr<const CostmapCloud>> marking_clouds;
    if (getMarkingClouds(marking_clouds)) {
      // LOG(INFO) << "get Negative Obstacles cloud point:";
      has_update_hash_.clear();
      markNegativeObstacles(marking_clouds);

      clearNegativeObstacles(marking_clouds);
    }
  }

  return true;
}

void NegativeObstaclesLayer::markNegativeObstacles(
    const std::vector<std::shared_ptr<const CostmapCloud>> &marking_clouds) {
  int ix, iy;
  double inv_res_x = 1.0 / d_resolution_;
  double inv_res_y = 1.0 / d_resolution_;
  size_t index = 0;
  size_t hash_num;
  for (const auto &cloud : marking_clouds) {
    // mark
    if (cloud->ptr_v_cloud_->size() > 0) {
      LOG(INFO) << "negative points: " << cloud->ptr_v_cloud_->size();
    } else {
      LOG(INFO) << "negative points empty.";
      continue;
    }
    for (const auto &point : *cloud->ptr_v_cloud_) {
      ix = (point.d_x - d_origin_x_) * inv_res_x;
      iy = (point.d_y - d_origin_y_) * inv_res_y;
      // LOG(INFO) << "point: " << point.d_x << ", " << point.d_y
      //           << " origin: " << d_origin_x_ << ", " << d_origin_y_;
      // LOG(INFO) << "mark: " << ix << ", " << iy;
      index = iy * ui_size_x_ + ix;
      hash_num = ix * 137 + iy * 149;
      has_update_hash_.insert(hash_num);
      if (ptr_uc_costmap_[index] < uc_default_value_ + 10) {
        unsigned char update_value = ptr_uc_costmap_[index] + 2;
        // LOG(INFO) << (int)uc_default_value_ << " " << (int)update_value;
        if (update_value < uc_default_value_ + 10) {
          ptr_uc_costmap_[index] = update_value;
        } else {
          // LOG(INFO) << "neg";
          // ptr_uc_costmap_[index] = LETHAL_OBSTACLE;
          //增加范围
          {
            for (int i = -3; i <= 3; i++) {
              for (int j = -3; j <= 3; j++) {
                index = (iy + i) * ui_size_x_ + ix + j;
                ptr_uc_costmap_[index] = LETHAL_OBSTACLE;
              }
            }
          }
        }
      }
    }
  }
}

bool NegativeObstaclesLayer::clearNegativeObstacles(
    const std::vector<std::shared_ptr<const CostmapCloud>> &clear_clouds) {
  int ix, iy;
  double inv_res_x = 1.0 / d_resolution_;
  double inv_res_y = 1.0 / d_resolution_;
  size_t hash_num;
  for (const auto &cloud : clear_clouds) {
    auto iter = chear_areas_.find(cloud->s_topic_name_);
    std::vector<Eigen::Vector4f> clear_area;

    //判读是否已经记录过对应点云的清除区域，如无计算加入存储，以topic name为key
    if (iter == chear_areas_.end()) {
      if (getClearAreaInBase(cloud, clear_area)) {
        chear_areas_.insert(
            std::pair<std::string, std::vector<Eigen::Vector4f>>(
                cloud->s_topic_name_, clear_area));
      }
    } else {
      clear_area = iter->second;
    }
    if (clear_area.size() != 4) {
      LOG(ERROR) << "get clear area vertexs size ! = 4";
      return false;
    }
    // LOG(INFO) << "clear area vertex: (" << clear_area[0](0) << ", "
    //           << clear_area[0](1) << ") - (" << clear_area[1](0) << ", "
    //           << clear_area[1](1) << ") - (" << clear_area[2](0) << ", "
    //           << clear_area[2](1) << ") - (" << clear_area[3](0) << ", "
    //           << clear_area[3](1) << ")";
    //计算清除区域顶点信息转换到cost map 坐标；
    WorldmapPose wp_robot_pose = cloud->world_pose_;
    Eigen::Matrix4f T_wo;
    T_wo << std::cos(wp_robot_pose.d_yaw), -std::sin(wp_robot_pose.d_yaw), 0,
        wp_robot_pose.d_x, std::sin(wp_robot_pose.d_yaw),
        std::cos(wp_robot_pose.d_yaw), 0, wp_robot_pose.d_y, 0, 0, 1, 0, 0, 0,
        0, 1;
    std::vector<std::vector<float>> clear_area_vertexs_map_index;
    std::vector<float> vertex_map_index(2);
    Eigen::Vector4f w_clear_area_vertex;

    for (unsigned int i = 0; i < clear_area.size(); i++) {
      w_clear_area_vertex = T_wo * clear_area[i];
      ix = (w_clear_area_vertex(0) - d_origin_x_) * inv_res_x;
      iy = (w_clear_area_vertex(1) - d_origin_y_) * inv_res_y;

      if (ix < 0) {
        ix = 0;
      } else if (ix > ui_size_x_) {
        ix = ui_size_x_;
      }
      if (iy < 0) {
        iy = 0;
      } else if (iy > ui_size_x_) {
        iy = ui_size_y_;
      }
      // LOG(INFO) << "vertex (" << w_clear_area_vertex(0) << ", "
      //           << w_clear_area_vertex(1) << ") to costmap (" << ix << ", "
      //           << iy << ")";
      vertex_map_index[0] = ix;
      vertex_map_index[1] = iy;
      clear_area_vertexs_map_index.push_back(vertex_map_index);
    }

    int minX = clear_area_vertexs_map_index[0][0];
    int maxX = clear_area_vertexs_map_index[0][0];
    int minY = clear_area_vertexs_map_index[0][1];
    int maxY = clear_area_vertexs_map_index[0][1];
    for (unsigned int i = 1; i < clear_area_vertexs_map_index.size(); i++) {
      if (minX > clear_area_vertexs_map_index[i][0]) {
        minX = clear_area_vertexs_map_index[i][0];
      }
      if (maxX < clear_area_vertexs_map_index[i][0]) {
        maxX = clear_area_vertexs_map_index[i][0];
      }
      if (minY > clear_area_vertexs_map_index[i][1]) {
        minY = clear_area_vertexs_map_index[i][1];
      }
      if (maxY < clear_area_vertexs_map_index[i][1]) {
        maxY = clear_area_vertexs_map_index[i][1];
      }
    }
    // LOG(INFO) << "clear: (" << minX << ", " << minY << ") - (" << maxX << ",
    // "
    //           << maxY << ")";
    size_t index;
    double dix, diy;
    size_t clear_count = 0;
    for (int iy = minY; iy < maxY; iy++) {
      for (int ix = minX; ix < maxX; ix++) {
        index = iy * ui_size_x_ + ix;
        if (ptr_uc_costmap_[index] == LETHAL_OBSTACLE) {
          continue;
        }
        if (ptr_uc_costmap_[index] > uc_default_value_) {
          if (isPointInPolygon(ix, iy, clear_area_vertexs_map_index)) {
            hash_num = ix * 137 + iy * 149;
            if (has_update_hash_.find(hash_num) == has_update_hash_.end()) {
              clear_count++;
              if (ptr_uc_costmap_[index] < uc_default_value_ + 2) {
                ptr_uc_costmap_[index] = uc_default_value_;
              } else {
                if (ptr_uc_costmap_[index] < uc_default_value_ + 10) {
                  ptr_uc_costmap_[index] -= 1;
                } else {
                  ptr_uc_costmap_[index] = uc_default_value_ + 9;
                }
              }
            }
          }
        }
      }
    }
    LOG(INFO) << "clear: " << clear_count;
  }
}

bool NegativeObstaclesLayer::getClearAreaInBase(
    const std::shared_ptr<const CostmapCloud> &cloud,
    std::vector<Eigen::Vector4f> &clear_area) {
  clear_area.clear();
  SensorPose sensor_pose = cloud->sensor_pose_;
  double min_d = cloud->negative_min_d_;
  double max_d = cloud->negative_max_d_;
  double v_fov = cloud->v_fov_;
  double h_fov = cloud->h_fov_;
  double v_fov_2 = v_fov / 2;
  double h_fov_2 = h_fov / 2;
  LOG(INFO) << "sensor_pose: " << sensor_pose.x << ", " << sensor_pose.y << ", "
            << sensor_pose.z << ", " << sensor_pose.yaw << ", "
            << sensor_pose.pitch << ", " << sensor_pose.roll;
  Eigen::Matrix4f T_os = Eigen::Matrix4f::Identity();
  T_os.block<3, 3>(0, 0) = Eigen::Matrix3f(
      Eigen::AngleAxisf(sensor_pose.yaw, Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(sensor_pose.pitch, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(sensor_pose.roll, Eigen::Vector3f::UnitX()));
  T_os(0, 3) = sensor_pose.x;
  T_os(1, 3) = sensor_pose.y;
  T_os(2, 3) = sensor_pose.z;

  double tan_v_fov_1 = std::tan(-v_fov_2);
  double tan_v_fov_2 = std::tan(v_fov_2);
  double tan_h_fov_1 = std::tan(-h_fov_2);
  double tan_h_fov_2 = std::tan(h_fov_2);

  // 计算相机坐标系下FOV 6面体的8个顶点

  /*
     /\ 6
  7 /  \
    |\  \  2
    | \/ |
    | 3| |
    | 0|/  1
    | / /
   4|/-/5

  */

  std::vector<Eigen::Vector4f> vertexs(8);
  vertexs[0] << min_d * tan_h_fov_1, min_d * tan_v_fov_1, min_d, 1;
  vertexs[1] << min_d * tan_h_fov_2, min_d * tan_v_fov_1, min_d, 1;
  vertexs[2] << min_d * tan_h_fov_2, min_d * tan_v_fov_2, min_d, 1;
  vertexs[3] << min_d * tan_h_fov_1, min_d * tan_v_fov_2, min_d, 1;
  vertexs[4] << max_d * tan_h_fov_1, max_d * tan_v_fov_1, max_d, 1;
  vertexs[5] << max_d * tan_h_fov_2, max_d * tan_v_fov_1, max_d, 1;
  vertexs[6] << max_d * tan_h_fov_2, max_d * tan_v_fov_2, max_d, 1;
  vertexs[7] << max_d * tan_h_fov_1, max_d * tan_v_fov_2, max_d, 1;
  for (unsigned int i = 0; i < vertexs.size(); i++) {
    LOG(INFO) << i << " " << vertexs[0];
  }
  //统计低于地面的定位数量
  unsigned int under_ground_vertex_count = 0;

  //转换到base_link 坐标系
  std::vector<Eigen::Vector4f> b_vertexs(8);
  for (unsigned int i = 0; i < vertexs.size(); i++) {
    b_vertexs[i] = T_os * vertexs[i];
    if (b_vertexs[i](2) < 0) {
      under_ground_vertex_count++;
      LOG(INFO) << "vertex: " << i << " under ground.";
    }
  }

  if (under_ground_vertex_count < 2 || under_ground_vertex_count > 8) {
    LOG(INFO) << "under ground vertex < 2 or > 8";
    return false;
  }

  if (under_ground_vertex_count == 2) {
    LOG(INFO) << "under ground vertex count was 2.";
    if (b_vertexs[4](2) < 0 && b_vertexs[5](2) < 0) {
      Eigen::Vector4f clear_area_vertex;
      if (getLineIntersectWhitGround(b_vertexs[4], b_vertexs[7],
                                     clear_area_vertex)) {
        clear_area.push_back(clear_area_vertex);
        LOG(INFO) << "line intersect whit ground 4-7: (" << b_vertexs[4](0)
                  << ", " << b_vertexs[4](1) << ", " << b_vertexs[4](2)
                  << ") - (" << b_vertexs[7](0) << ", " << b_vertexs[7](1)
                  << ", " << b_vertexs[7](2) << ")  -> ("
                  << clear_area_vertex(0) << ", " << clear_area_vertex(1)
                  << ", " << clear_area_vertex(2) << ")";
      } else {
        LOG(ERROR) << "get line intersect whit ground fail 4 - 7";
        return false;
      }

      if (getLineIntersectWhitGround(b_vertexs[4], b_vertexs[0],
                                     clear_area_vertex)) {
        clear_area.push_back(clear_area_vertex);
        LOG(INFO) << "line intersect whit ground 4-0: (" << b_vertexs[4](0)
                  << ", " << b_vertexs[4](1) << ", " << b_vertexs[4](2)
                  << ") - (" << b_vertexs[0](0) << ", " << b_vertexs[0](1)
                  << ", " << b_vertexs[0](2) << ")  -> ("
                  << clear_area_vertex(0) << ", " << clear_area_vertex(1)
                  << ", " << clear_area_vertex(2) << ")";
      } else {
        LOG(ERROR) << "get line intersect whit ground fail 4 - 0";
        return false;
      }

      if (getLineIntersectWhitGround(b_vertexs[5], b_vertexs[1],
                                     clear_area_vertex)) {
        clear_area.push_back(clear_area_vertex);
        LOG(INFO) << "line intersect whit ground 5-1: (" << b_vertexs[5](0)
                  << ", " << b_vertexs[5](1) << ", " << b_vertexs[5](2)
                  << ") - (" << b_vertexs[1](0) << ", " << b_vertexs[1](1)
                  << ", " << b_vertexs[1](2) << ")  -> ("
                  << clear_area_vertex(0) << ", " << clear_area_vertex(1)
                  << ", " << clear_area_vertex(2) << ")";
      } else {
        LOG(ERROR) << "get line intersect whit ground fail 5 - 1";
        return false;
      }

      if (getLineIntersectWhitGround(b_vertexs[5], b_vertexs[6],
                                     clear_area_vertex)) {
        clear_area.push_back(clear_area_vertex);
        LOG(INFO) << "line intersect whit ground 5-6: (" << b_vertexs[5](0)
                  << ", " << b_vertexs[5](1) << ", " << b_vertexs[5](2)
                  << ") - (" << b_vertexs[6](0) << ", " << b_vertexs[6](1)
                  << ", " << b_vertexs[6](2) << ")  -> ("
                  << clear_area_vertex(0) << ", " << clear_area_vertex(1)
                  << ", " << clear_area_vertex(2) << ")";

      } else {
        LOG(ERROR) << "get line intersect whit ground fail 5 - 6";
        return false;
      }
    } else if (b_vertexs[6](2) < 0 && b_vertexs[7](2) < 0) {
      Eigen::Vector4f clear_area_vertex;
      if (getLineIntersectWhitGround(b_vertexs[7], b_vertexs[4],
                                     clear_area_vertex)) {
        clear_area.push_back(clear_area_vertex);
        LOG(INFO) << "line intersect whit ground 7-4: (" << b_vertexs[7](0)
                  << ", " << b_vertexs[7](1) << ", " << b_vertexs[7](2)
                  << ") - (" << b_vertexs[4](0) << ", " << b_vertexs[4](1)
                  << ", " << b_vertexs[4](2) << ")  -> ("
                  << clear_area_vertex(0) << ", " << clear_area_vertex(1)
                  << ", " << clear_area_vertex(2) << ")";
      } else {
        LOG(ERROR) << "get line intersect whit ground fail 7 - 4";
        return false;
      }

      if (getLineIntersectWhitGround(b_vertexs[7], b_vertexs[3],
                                     clear_area_vertex)) {
        clear_area.push_back(clear_area_vertex);
        LOG(INFO) << "line intersect whit ground 7-3: (" << b_vertexs[7](0)
                  << ", " << b_vertexs[7](1) << ", " << b_vertexs[7](2)
                  << ") - (" << b_vertexs[3](0) << ", " << b_vertexs[3](1)
                  << ", " << b_vertexs[3](2) << ")  -> ("
                  << clear_area_vertex(0) << ", " << clear_area_vertex(1)
                  << ", " << clear_area_vertex(2) << ")";
      } else {
        LOG(ERROR) << "get line intersect whit ground fail 7 - 3";
        return false;
      }

      if (getLineIntersectWhitGround(b_vertexs[6], b_vertexs[2],
                                     clear_area_vertex)) {
        clear_area.push_back(clear_area_vertex);
        LOG(INFO) << "line intersect whit ground 6-2: (" << b_vertexs[6](0)
                  << ", " << b_vertexs[6](1) << ", " << b_vertexs[6](2)
                  << ") - (" << b_vertexs[2](0) << ", " << b_vertexs[2](1)
                  << ", " << b_vertexs[2](2) << ")  -> ("
                  << clear_area_vertex(0) << ", " << clear_area_vertex(1)
                  << ", " << clear_area_vertex(2) << ")";
      } else {
        LOG(ERROR) << "get line intersect whit ground fail 6 - 2";
        return false;
      }

      if (getLineIntersectWhitGround(b_vertexs[6], b_vertexs[5],
                                     clear_area_vertex)) {
        clear_area.push_back(clear_area_vertex);
        LOG(INFO) << "line intersect whit ground 6-5: (" << b_vertexs[6](0)
                  << ", " << b_vertexs[6](1) << ", " << b_vertexs[6](2)
                  << ") - (" << b_vertexs[5](0) << ", " << b_vertexs[5](1)
                  << ", " << b_vertexs[5](2) << ")  -> ("
                  << clear_area_vertex(0) << ", " << clear_area_vertex(1)
                  << ", " << clear_area_vertex(2) << ")";
      } else {
        LOG(ERROR) << "get line intersect whit ground fail 6 - 5";
        return false;
      }
    } else {
      LOG(INFO) << "under ground vertex = 2, but not able to handle now.";
      return false;
    }
  } else if (under_ground_vertex_count == 4) {
    if ((b_vertexs[0](2) < 0 && b_vertexs[1](2) < 0 && b_vertexs[4](2) < 0 &&
         b_vertexs[5](2) < 0) ||
        (b_vertexs[2](2) < 0 && b_vertexs[3](2) < 0 && b_vertexs[6](2) < 0 &&
         b_vertexs[7](2))) {
      Eigen::Vector4f clear_area_vertex;
      if (getLineIntersectWhitGround(b_vertexs[4], b_vertexs[7],
                                     clear_area_vertex)) {
        clear_area.push_back(clear_area_vertex);
        LOG(INFO) << "line intersect whit ground 4-7: (" << b_vertexs[4](0)
                  << ", " << b_vertexs[4](1) << ", " << b_vertexs[4](2)
                  << ") - (" << b_vertexs[7](0) << ", " << b_vertexs[7](1)
                  << ", " << b_vertexs[7](2) << ")  -> ("
                  << clear_area_vertex(0) << ", " << clear_area_vertex(1)
                  << ", " << clear_area_vertex(2) << ")";

      } else {
        LOG(ERROR) << "get line intersect whit ground fail 4 - 7";
        return false;
      }

      if (getLineIntersectWhitGround(b_vertexs[0], b_vertexs[3],
                                     clear_area_vertex)) {
        clear_area.push_back(clear_area_vertex);
        LOG(INFO) << "line intersect whit ground 0-3: (" << b_vertexs[0](0)
                  << ", " << b_vertexs[0](1) << ", " << b_vertexs[0](2)
                  << ") - (" << b_vertexs[3](0) << ", " << b_vertexs[3](1)
                  << ", " << b_vertexs[3](2) << ")  -> ("
                  << clear_area_vertex(0) << ", " << clear_area_vertex(1)
                  << ", " << clear_area_vertex(2) << ")";
      } else {
        LOG(ERROR) << "get line intersect whit ground fail 0 - 3";
        return false;
      }

      if (getLineIntersectWhitGround(b_vertexs[1], b_vertexs[2],
                                     clear_area_vertex)) {
        clear_area.push_back(clear_area_vertex);
        LOG(INFO) << "line intersect whit ground 1-2: (" << b_vertexs[1](0)
                  << ", " << b_vertexs[1](1) << ", " << b_vertexs[1](2)
                  << ") - (" << b_vertexs[2](0) << ", " << b_vertexs[2](1)
                  << ", " << b_vertexs[2](2) << ")  -> ("
                  << clear_area_vertex(0) << ", " << clear_area_vertex(1)
                  << ", " << clear_area_vertex(2) << ")";
      } else {
        LOG(ERROR) << "get line intersect whit ground fail 1 - 2";
        return false;
      }

      if (getLineIntersectWhitGround(b_vertexs[5], b_vertexs[6],
                                     clear_area_vertex)) {
        clear_area.push_back(clear_area_vertex);
        LOG(INFO) << "line intersect whit ground 5-6: (" << b_vertexs[5](0)
                  << ", " << b_vertexs[5](1) << ", " << b_vertexs[5](2)
                  << ") - (" << b_vertexs[6](0) << ", " << b_vertexs[6](1)
                  << ", " << b_vertexs[6](2) << ")  -> ("
                  << clear_area_vertex(0) << ", " << clear_area_vertex(1)
                  << ", " << clear_area_vertex(2) << ")";
      } else {
        LOG(ERROR) << "get line intersect whit ground fail 5 - 6";
        return false;
      }
    } else {
      LOG(INFO) << "under ground vertex = 4, but not able to handle now.";
      return false;
    }

  } else if (under_ground_vertex_count == 6) {
    LOG(INFO) << "under ground vertex = 6, but not able to handle now.";
    return false;
  } else {
    LOG(ERROR) << "under ground vertex count was singular";
    return false;
  }
  return true;
}

bool NegativeObstaclesLayer::getLineIntersectWhitGround(
    const Eigen::Vector4f &p1, const Eigen::Vector4f &p2,
    Eigen::Vector4f &intersect_point) {
  if (p1(2) * p2(2) >= 0) {
    LOG(INFO) << "can not get the point intersect whit ground in line(" << p1(0)
              << ", " << p1(1) << ", " << p1(2) << ") - (" << p2(0) << ", "
              << p2(1) << ", " << p2(2) << ")";
    return false;
  }
  float a = std::fabs(p1(2)) / (std::fabs(p1(2)) + std::fabs(p2(2)));
  intersect_point(0) = p1(0) + a * (p2(0) - p1(0));
  intersect_point(1) = p1(1) + a * (p2(1) - p1(1));
  intersect_point(2) = 0;
  intersect_point(3) = 1;
}

bool NegativeObstaclesLayer::isPointInPolygon(
    const int x, const int y, const std::vector<std::vector<float>> &points) {
  // vector<Point> points:表示闭合区域由这些点围成
  float minX = points[0][0];
  float maxX = points[0][0];
  float minY = points[0][1];
  float maxY = points[0][1];
  for (unsigned int i = 1; i < points.size(); i++) {
    minX = std::min(points[i][0], minX);
    maxX = std::max(points[i][0], maxX);
    minY = std::min(points[i][1], minY);
    maxY = std::max(points[i][1], maxY);
  }

  if (x < minX || x > maxX || y < minY || y > maxY) {
    return false;
  }

  bool inside = false;
  for (unsigned int i = 0, j = points.size() - 1; i < points.size(); j = i++) {
    if ((points[i][1] > y) != (points[j][1] > y) &&
        x < (points[j][0] - points[i][0]) * (y - points[i][1]) /
                    (points[j][1] - points[i][1]) +
                points[i][0]) {
      inside = !inside;
    }
  }

  return inside;
}

bool NegativeObstaclesLayer::getMarkingClouds(
    std::vector<std::shared_ptr<const CostmapCloud>> &marking_clouds) const {
  std::shared_ptr<CostmapCloud> ptr_cloud = nullptr;
  for (const auto &topic_name : v_marking_cloud_names_) {
    if (!CostmapMediator::getPtrInstance()->getData(topic_name, ptr_cloud)) {
      LOG(ERROR) << topic_name << " not regitstered.";
      continue;
    }
    // 如果点云已经处理过，不需要重复处理
    if (ptr_cloud->is_used_to_mark_) {
      continue;
    }
    // 标记当前访问点云已经被处理过，避免重复处理
    ptr_cloud->is_used_to_mark_ = true;

    std::shared_ptr<CostmapCloud> transformed_cloud;
    if (ptr_cloud->b_need_transformed_) {
      transformed_cloud = ptr_cloud->transformToMap();
      marking_clouds.push_back(transformed_cloud);
    } else {
      marking_clouds.push_back(ptr_cloud);
    }
  }
  if (marking_clouds.size() > 0) {
    return true;
  } else {
    return false;
  }
}

bool NegativeObstaclesLayer::getClearingClouds(
    std::vector<std::shared_ptr<const CostmapCloud>> &clearing_clouds) const {
  return false;
}

void NegativeObstaclesLayer::matchSize() {
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  ObstacleLayer::matchSize();
}

void NegativeObstaclesLayer::resetMaps() {
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  Costmap2d::resetMaps();
}

void NegativeObstaclesLayer::activate() { b_enabled_ = true; }
void NegativeObstaclesLayer::deactivate() {
  b_enabled_ = false;
  reset();
}
void NegativeObstaclesLayer::reset() {
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  Costmap2d::resetMaps();
}

}  // namespace CVTE_BABOT