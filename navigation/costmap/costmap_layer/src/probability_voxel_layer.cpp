#include "probability_voxel_layer.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <unordered_set>

#include "costmap_mediator.hpp"
namespace CVTE_BABOT {
// TODO: 构造函数中初始化grid对象;
bool ProbabilityVoxelLayer::onInitialize() {
  if (!ObstacleLayer::onInitialize()) {
    LOG(ERROR) << "onInitialize failed";
    return false;
  }

  return true;
}

void ProbabilityVoxelLayer::getParams() {
  ObstacleLayer::getParams();
  if (!b_enabled_) {
    return;
  }

  CostmapMediator::getPtrInstance()->getParam(s_name_ + "." + "origin_z",
                                              origin_z_, 0.0);
  CostmapMediator::getPtrInstance()->getParam(s_name_ + "." + "z_resolution",
                                              resolution_z_, 0.1);
  CostmapMediator::getPtrInstance()->getParam(s_name_ + "." + "z_voxels",
                                              size_z_, 16);
  CostmapMediator::getPtrInstance()->getParam(
      s_name_ + "." + "unknown_threshold", unknown_threshold_, 10);
  CostmapMediator::getPtrInstance()->getParam(s_name_ + "." + "mark_threshold",
                                              mark_threshold_, 0);
  CostmapMediator::getPtrInstance()->getParam(
      s_name_ + "." + "clearing_time_threshold", clear_time_threshold_, 5.0);
}

void ProbabilityVoxelLayer::matchSize() {
  LOG(INFO) << "VoxelLayer matchSize";
  ObstacleLayer::matchSize();
  auto ptr_costmap = CostmapMediator::getPtrInstance()->getCostmap();
  int map_size_x = ptr_costmap->getSizeInCellsX();
  int map_size_y = ptr_costmap->getSizeInCellsY();
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  if (nullptr != ptr_grid_) {
    ptr_grid_.reset();
  }

  ptr_grid_ = std::make_shared<ProbabilityVoxelGrid>(
      map_size_x, map_size_y, size_z_, d_resolution_, d_resolution_,
      resolution_z_);
  LOG(INFO) << "VoxelLayer matchSize finished :" << map_size_x << " "
            << map_size_y << " " << size_z_;
}

void ProbabilityVoxelLayer::resetMaps() {
  Costmap2d::resetMaps();
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  if (nullptr != ptr_grid_) {
    ptr_grid_->reset();
  }
}

bool ProbabilityVoxelLayer::downSampleByGrid(
    const std::shared_ptr<const CostmapCloud> &raw_cloud,
    std::shared_ptr<CostmapCloud> &tar_cloud) const {
  if (!raw_cloud) {
    LOG(WARNING) << "passed to downsample cloud is null !";
    return false;
  }
  if (!tar_cloud) {
    tar_cloud.reset(new CostmapCloud);
  }

  int ix, iy, iz;
  CostmapPointXYZ vp;
  assert(fabs(ptr_grid_->resX()) > std::numeric_limits<double>::epsilon());
  assert(fabs(ptr_grid_->resY()) > std::numeric_limits<double>::epsilon());
  assert(fabs(ptr_grid_->resZ()) > std::numeric_limits<double>::epsilon());
  double inv_res_x = 1.0 / ptr_grid_->resX();
  double inv_res_y = 1.0 / ptr_grid_->resY();
  double inv_res_z = 1.0 / ptr_grid_->resZ();
  std::unordered_set<size_t> vps_hash;
  for (const auto &point : *raw_cloud->ptr_v_cloud_) {
    ix = (point.d_x - d_origin_x_) * inv_res_x;
    iy = (point.d_y - d_origin_y_) * inv_res_y;
    iz = (point.d_z - origin_z_) * inv_res_z;

    //用三个质数产生三维坐标的hash值
    size_t hash_value = 137 * ix + 149 * iy + 163 * iz;
    auto it = vps_hash.find(hash_value);

    if (it == vps_hash.end()) {
      vps_hash.insert(hash_value);
      (tar_cloud->ptr_v_cloud_)->push_back(point);
    }
  }

  // 复制重要信息
  tar_cloud->s_topic_name_ = raw_cloud->s_topic_name_;
  tar_cloud->obstacle_range_ = raw_cloud->obstacle_range_;
  tar_cloud->raytrace_range_ = raw_cloud->raytrace_range_;
  tar_cloud->v_fov_ = raw_cloud->v_fov_;
  tar_cloud->h_fov_ = raw_cloud->h_fov_;
  tar_cloud->min_d_ = raw_cloud->min_d_;
  tar_cloud->max_d_ = raw_cloud->max_d_;
  tar_cloud->origin_ = raw_cloud->origin_;
  tar_cloud->sensor_pose_ = raw_cloud->sensor_pose_;
  tar_cloud->world_pose_ = raw_cloud->world_pose_;
  tar_cloud->b_need_transformed_ = raw_cloud->b_need_transformed_;
  tar_cloud->is_used_to_mark_ = raw_cloud->is_used_to_mark_;
  tar_cloud->is_used_to_clear_ = raw_cloud->is_used_to_clear_;

  return true;
}

bool ProbabilityVoxelLayer::getMarkingClouds(
    std::vector<std::shared_ptr<const CostmapCloud>> &v_ptr_clouds) const {
  bool b_current = true;

  std::shared_ptr<CostmapCloud> ptr_cloud = nullptr;
  for (const auto &topic_name : v_marking_cloud_names_) {
    if (!CostmapMediator::getPtrInstance()->getData(topic_name, ptr_cloud)) {
      LOG(ERROR) << topic_name << " not regitstered.";
      b_current = false;
      continue;
    }

    // 如果点云已经处理过，不需要重复处理
    if (ptr_cloud->is_used_to_mark_) {
      continue;
    }

    // 标记当前访问点云已经被处理过，避免重复处理
    ptr_cloud->is_used_to_mark_ = true;

    // 保存有效点云
    v_ptr_clouds.push_back(ptr_cloud);
  }

  return b_current;
}

bool ProbabilityVoxelLayer::getClearingClouds(
    std::vector<std::shared_ptr<const CostmapCloud>> &v_ptr_clouds) const {
  bool b_current = true;

  std::shared_ptr<CostmapCloud> ptr_cloud = nullptr;
  for (const auto &topic_name : v_clearing_cloud_names_) {
    if (!CostmapMediator::getPtrInstance()->getData(topic_name, ptr_cloud)) {
      LOG(ERROR) << topic_name << " not regitstered.";
      b_current = false;
      continue;
    }

    // 如果点云已经处理过，不需要重复处理
    if (ptr_cloud->is_used_to_clear_) {
      continue;
    }
    // 标记当前访问点云已经被处理过，避免重复处理
    ptr_cloud->is_used_to_clear_ = true;

    // 保存有效点云
    v_ptr_clouds.push_back(ptr_cloud);
  }

  return b_current;
}

bool ProbabilityVoxelLayer::updateBounds(const WorldmapPose &wp_robot_pose,
                                         CostmapBound &cb_costmap_bound) {
  // LOG(INFO) << "VoxelLayer updateBounds";
  if (!b_enabled_) {
    LOG(INFO) << "VoxeLayer not enable";
    return false;
  }
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  if (b_rolling_window_) {
    updateOrigin(wp_robot_pose.d_x - getSizeInMetersX() / 2,
                 wp_robot_pose.d_y - getSizeInMetersY() / 2);
  }
  useExtraBounds(cb_costmap_bound);
  if (getLayerUpdate()) {  ///< 只有体素层，负向有这个标志位，为了打开或关闭这层，由navigation传入
    std::vector<std::shared_ptr<const CostmapCloud>> marking_clouds,
        clearing_clouds;

    getMarkingClouds(marking_clouds);
    // 判断"清除点云"和"标记点云"是否相同，避免重复获取浪费资源
    if (v_marking_cloud_names_ == v_clearing_cloud_names_) {
      clearing_clouds = marking_clouds;
    } else {
      getClearingClouds(clearing_clouds);
    }
    time_t now_time = time(NULL);
    static time_t last_time = now_time;
    if (now_time - last_time > 5.0) {
      LOG(INFO) << "clear by time.";
      clearByTime(now_time, clear_time_threshold_);
      last_time = now_time;
    }

    ptr_grid_->startUpate();
    if (marking_clouds.size() > 0) {
      // LOG(INFO) << "markVoxelMap";
      markVoxelMap(marking_clouds, now_time);
    }

    if (clearing_clouds.size() > 0) {
      // LOG(INFO) << "clearFrustums";
      clearFrustums(clearing_clouds);
    }
  }
  updateCostsMap();
  updateFootprint(wp_robot_pose, cb_costmap_bound);

  return true;
}

void ProbabilityVoxelLayer::updateOrigin(const double &d_new_origin_x,
                                         const double &d_new_origin_y) {
  int cell_ox =
      static_cast<int>((d_new_origin_x - d_origin_x_) / d_resolution_);
  int cell_oy =
      static_cast<int>((d_new_origin_y - d_origin_y_) / d_resolution_);
  double new_grid_ox = d_origin_x_ + cell_ox * d_resolution_;
  double new_grid_oy = d_origin_y_ + cell_oy * d_resolution_;

  ptr_grid_->shifting(cell_ox, cell_oy);

  d_origin_x_ = new_grid_ox;
  d_origin_y_ = new_grid_oy;

  // TODO: check is 2d map need to update here
}

void ProbabilityVoxelLayer::clearFrustums(
    const std::vector<std::shared_ptr<const CostmapCloud>> &clearing_cloud) {
  if (clearing_cloud.empty()) {
    return;
  }
  size_t unmark_count = 0;
  for (size_t i = 0; i < clearing_cloud.size(); i++) {
    // 提前计算当前点云参数，避免后面重复计算
    WorldmapPose wp_robot_pose = clearing_cloud[i]->world_pose_;
    SensorPose sensor_pose = clearing_cloud[i]->sensor_pose_;
    double min_d = clearing_cloud[i]->min_d_;
    double max_d = clearing_cloud[i]->max_d_;
    double min_h = clearing_cloud[i]->min_h_;
    double max_h = clearing_cloud[i]->max_h_;
    double v_fov = clearing_cloud[i]->v_fov_;
    double h_fov = clearing_cloud[i]->h_fov_;

    double v_fov_2 = v_fov / 2;
    double h_fov_2 = h_fov / 2;

    Eigen::Matrix4f T_wo;

    // 计算传感器坐标系相对于世界坐标系的位姿
    T_wo << std::cos(wp_robot_pose.d_yaw), -std::sin(wp_robot_pose.d_yaw), 0,
        wp_robot_pose.d_x, std::sin(wp_robot_pose.d_yaw),
        std::cos(wp_robot_pose.d_yaw), 0, wp_robot_pose.d_y, 0, 0, 1, 0, 0, 0,
        0, 1;

    Eigen::Matrix4f T_os = Eigen::Matrix4f::Identity();
    T_os.block<3, 3>(0, 0) = Eigen::Matrix3f(
        Eigen::AngleAxisf(sensor_pose.yaw, Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(sensor_pose.pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(sensor_pose.roll, Eigen::Vector3f::UnitX()));
    T_os(0, 3) = sensor_pose.x;
    T_os(1, 3) = sensor_pose.y;
    T_os(2, 3) = sensor_pose.z;

    Eigen::Matrix4f T_ws = T_wo * T_os;
    Eigen::Matrix4f T_ws_inverse = T_ws.inverse();

    double tan_v_fov_1 = std::tan(-v_fov_2);
    double tan_v_fov_2 = std::tan(v_fov_2);
    double tan_h_fov_1 = std::tan(-h_fov_2);
    double tan_h_fov_2 = std::tan(h_fov_2);

    // 计算相机坐标系下FOV 6面体的8个顶点
    std::vector<Eigen::Vector4f> vertexs(8);
    vertexs[0] << min_d * tan_h_fov_1, min_d * tan_v_fov_1, min_d, 1;
    vertexs[1] << min_d * tan_h_fov_1, min_d * tan_v_fov_2, min_d, 1;
    vertexs[2] << min_d * tan_h_fov_2, min_d * tan_v_fov_2, min_d, 1;
    vertexs[3] << min_d * tan_h_fov_2, min_d * tan_v_fov_1, min_d, 1;
    vertexs[4] << max_d * tan_h_fov_1, max_d * tan_v_fov_1, max_d, 1;
    vertexs[5] << max_d * tan_h_fov_1, max_d * tan_v_fov_2, max_d, 1;
    vertexs[6] << max_d * tan_h_fov_2, max_d * tan_v_fov_2, max_d, 1;
    vertexs[7] << max_d * tan_h_fov_2, max_d * tan_v_fov_1, max_d, 1;

    //计算在世界坐标系下,8个顶点最大最小xyz
    // TODO: 判读是否在有效范围内
    Eigen::Vector4f w_vertex;
    w_vertex = T_ws * vertexs[0];
    float min_x = w_vertex[0];
    float max_x = w_vertex[0];
    float min_y = w_vertex[1];
    float max_y = w_vertex[1];
    float min_z = w_vertex[2];
    float max_z = w_vertex[2];

    for (unsigned int vi = 1; vi < vertexs.size(); vi++) {
      w_vertex = T_ws * vertexs[vi];
      min_x = std::min(min_x, w_vertex[0]);
      max_x = std::max(max_x, w_vertex[0]);
      min_y = std::min(min_y, w_vertex[1]);
      max_y = std::max(max_y, w_vertex[1]);
      min_z = std::min(min_z, w_vertex[2]);
      max_z = std::max(max_z, w_vertex[2]);
    }
    int min_bx = (min_x - d_origin_x_) / ptr_grid_->resX();
    int max_bx = (max_x - d_origin_x_) / ptr_grid_->resX();
    int min_by = (min_y - d_origin_y_) / ptr_grid_->resY();
    int max_by = (max_y - d_origin_y_) / ptr_grid_->resY();
    size_t clear_count = 0;
    time_t time_now = time(NULL);
    double wpx, wpy, wpz;
    float tmp_ix, tmp_iy, tmp_iz;
    bool in_h_fov;
    // LOG(INFO) << "ix: " << size_t(min_bx) << " - " << size_t(max_bx);
    // LOG(INFO) << "iy: " << size_t(min_by) << " - " << size_t(max_by);
    Eigen::Vector4f t_wp;
    Eigen::Vector4f t_sp;
    double angle_h;
    double angle_v;
    bool mask;
    int angel_hash;
    int i_min_h = min_h / ptr_grid_->resZ();
    int i_max_h = max_h / ptr_grid_->resZ();

    for (size_t iy = size_t(min_by); iy < size_t(max_by); iy++) {
      for (size_t ix = size_t(min_bx); ix < size_t(max_bx); ix++) {
        // bool last_grid_clean = false;
        for (size_t iz = 0; iz < ptr_grid_->sizeZ(); iz++) {
          if (iz < i_min_h || iz > i_max_h) {
            continue;
          }
          if (ptr_grid_->getValue(ix, iy, iz) < 0.3) {
            // ptr_grid_->updateTime(ix, iy, time_now);
            continue;
          }

          // TODO: 此处可优化计算， 减少点云转换，尝试改用判断点是否在范围内
          wpx = d_origin_x_ + ix * ptr_grid_->resX();
          wpy = d_origin_y_ + iy * ptr_grid_->resY();
          wpz = origin_z_ + iz * ptr_grid_->resZ();

          t_wp << wpx, wpy, wpz, 1;
          t_sp = T_ws_inverse * t_wp;

          // 检查xy是否在传感器观测范围内
          if (t_sp[2] < min_d || t_sp[2] > max_d) {
            continue;
          }
          //低矮障碍检测有效清除距离改为1m
          // if (iz == 0 && t_sp[2] > 1) {
          //   continue;
          // }

          angle_h = std::atan2(t_sp[0], t_sp[2]);
          // FIXME: fov是否对应增加上下取整的补充?
          if (angle_h < -h_fov_2 || angle_h > h_fov_2) {
            // LOG(INFO)<<"not clear angle_h: "<< angle_h;
            continue;
          }

          angle_v = std::atan2(t_sp[1], t_sp[2]);
          if (angle_v > -v_fov_2 && angle_v < v_fov_2) {
            ptr_grid_->unmarkVoxel(ix, iy, iz, time_now);

          } else {
            // 如果点在FOV上方，且前一个（下一格）被清除了，那么这一个格也进行清除操作
            // 解决机器人在移动过程中接近障碍物并停下后，障碍物移动，清除不干净的问题
            // if (last_grid_clean) {
            //   ptr_grid_->unmarkVoxel(ix, iy, iz, time_now);
            //   last_grid_clean = true;
            // } else {
            //   last_grid_clean = false;
            // }
          }
        }
      }
    }
  }

  // LOG(INFO) << "unmark: " << unmark_count;
}

void ProbabilityVoxelLayer::markVoxelMap(
    const std::vector<std::shared_ptr<const CostmapCloud>> &mark_cloud,
    const time_t &mark_time) {
  int ix, iy, iz;
  size_t mark_count = 0;
  double inv_res_x = 1.0 / ptr_grid_->resX();
  double inv_res_y = 1.0 / ptr_grid_->resY();
  double inv_res_z = 1.0 / ptr_grid_->resZ();

  for (const auto &cloud : mark_cloud) {
    // 提前计算当前点云参数，避免后面重复计算
    WorldmapPose wp_robot_pose = cloud->world_pose_;
    SensorPose sensor_pose = cloud->sensor_pose_;
    double min_d = cloud->min_d_;
    double max_d = cloud->max_d_;
    double min_h = cloud->min_h_;
    double max_h = cloud->max_h_;
    double v_fov = cloud->v_fov_;
    double h_fov = cloud->h_fov_;
    double v_fov_2 = v_fov / 2;
    double h_fov_2 = h_fov / 2;

    // // 计算传感器坐标系相对于世界坐标系的位姿
    Eigen::Matrix4f T_wo;
    T_wo << std::cos(wp_robot_pose.d_yaw), -std::sin(wp_robot_pose.d_yaw), 0,
        wp_robot_pose.d_x, std::sin(wp_robot_pose.d_yaw),
        std::cos(wp_robot_pose.d_yaw), 0, wp_robot_pose.d_y, 0, 0, 1, 0, 0, 0,
        0, 1;

    Eigen::Matrix4f T_os = Eigen::Matrix4f::Identity();
    T_os.block<3, 3>(0, 0) = Eigen::Matrix3f(
        Eigen::AngleAxisf(sensor_pose.yaw, Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(sensor_pose.pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(sensor_pose.roll, Eigen::Vector3f::UnitX()));
    T_os(0, 3) = sensor_pose.x;
    T_os(1, 3) = sensor_pose.y;
    T_os(2, 3) = sensor_pose.z;

    Eigen::Matrix4f T_ws = T_wo * T_os;

    // mark有效点云
    Eigen::Vector4f scp;
    Eigen::Vector4f wcp;
    double angle_h;
    double angle_v;
    double wpx;
    double wpy;
    double wpz;
    for (const auto &point_org : *cloud->ptr_v_cloud_) {
      scp << point_org.d_x, point_org.d_y, point_org.d_z, 1;
      wcp = T_ws * scp;  /// 点转换到世界坐标

      //--------------------------------------清除无效数据----
      //---相当于滤波，box滤波，边界来源于实际需要，高度不超过机器人高度，最低为需要检测的高度
      if (wcp[2] < min_h || wcp[2] > max_h) {
        continue;
      }
      if (point_org.d_z < min_d || point_org.d_z > max_d) {
        // LOG(INFO)<<"not mark d: "<< t_sp[2];
        continue;
      }
      angle_h = std::atan2(point_org.d_x, point_org.d_z);
      if (angle_h < -h_fov_2 || angle_h > h_fov_2) {
        // LOG(INFO)<<"not mark angle_h: "<< angle_h;
        continue;
      }

      angle_v = std::atan2(point_org.d_y, point_org.d_z);
      if (angle_v < -v_fov_2 || angle_v > v_fov_2) {
        // LOG(INFO)<<"not mark angle_v: "<< angle_v;
        continue;
      }

      if (wcp[2] > d_max_obstacle_height_ || wcp[2] < d_min_obstacle_height_ ||
          wcp[2] < origin_z_) {
        continue;
      }

      //------------------------------------------清除无效数据------

      // 计算点云索引
      ix = (wcp[0] - d_origin_x_) * inv_res_x;
      iy = (wcp[1] - d_origin_y_) * inv_res_y;
      iz = (wcp[2] - origin_z_) * inv_res_z;

      // 确定索引栅格是否在fov内
      wpx = d_origin_x_ + ix * ptr_grid_->resX();
      wpy = d_origin_y_ + iy * ptr_grid_->resY();
      wpz = origin_z_ + iz * ptr_grid_->resZ();

      Eigen::Vector4f t_wp(wpx, wpy, wpz, 1);
      Eigen::Vector4f t_sp = T_ws.inverse() * t_wp;

      if (t_sp[2] < min_d || t_sp[2] > max_d) {
        // LOG(INFO) << "not mark d: " << t_sp[2];
        continue;
      }
      //低矮障碍物检测距离为1.0m
      //  if (iz == 0 && t_sp[2] > 1) {
      // continue;
      // }

      angle_h = std::atan2(t_sp[0], t_sp[2]);
      if (angle_h < -h_fov_2 || angle_h > h_fov_2) {
        // LOG(INFO) << "not mark angle_h: " << angle_h;
        continue;
      }

      angle_v = std::atan2(t_sp[1], t_sp[2]);
      if (angle_v < -v_fov_2 || angle_v > v_fov_2) {
        // LOG(INFO) << "not mark angle_v: " << angle_v;
        continue;
      }
      // TODO: -----以上可以换一种策略-------------

      // LOG(INFO)<<"mark grid: ("<<ix<<", "<< iy<<", "<< iz<<")";
      ptr_grid_->markVoxel(ix, iy, iz, mark_time);
      // if (iz == 0) {
      //   ptr_grid_->markVoxel(ix, iy, iz, mark_time);
      //   ptr_grid_->markVoxel(ix, iy, iz, mark_time);
      // }
      mark_count++;
    }
  }
  // LOG(INFO) << "mark: " << mark_count;
}

void ProbabilityVoxelLayer::clearByTime(const time_t &now_time,
                                        const double &clearing_time_threshold) {
  ptr_grid_->clearByTime(now_time, clearing_time_threshold);
}

void ProbabilityVoxelLayer::updateCostsMap() {
  if (b_enabled_) {
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    ptr_grid_->updateCostMap(ptr_uc_costmap_, LETHAL_OBSTACLE, 50, FREE_SPACE);
  }
}

void ProbabilityVoxelLayer::activate() { b_enabled_ = true; }
void ProbabilityVoxelLayer::deactivate() {
  b_enabled_ = false;
  reset();
}
void ProbabilityVoxelLayer::reset() {
  if (nullptr != ptr_grid_) {
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    ptr_grid_->reset();
  }
}
}  // namespace CVTE_BABOT