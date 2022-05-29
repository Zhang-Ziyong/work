/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file range_sensor_layer.cpp
 *
 *@brief range sensor layer 具体实现.
 *
 *@modified by chennuo(chennuo@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-09
 ************************************************************************/
#include <glog/logging.h>
#include <sstream>

#include "costmap_mediator.hpp"
#include "costmap_range_data.hpp"
#include "footprint_utils.hpp"
#include "layer.hpp"
#include "range_sensor_layer.hpp"

namespace CVTE_BABOT {
RangeSensorLayer::RangeSensorLayer() {
  b_use_for_navigation_ = false;
  b_can_use_for_avoidance_ = true;
}

RangeSensorLayer::~RangeSensorLayer() {}

bool RangeSensorLayer::onInitialize() {
  if (!CostmapMediator::getPtrInstance()->isParameterReady()) {
    LOG(ERROR) << "Is mediator's Parameter initialized? ";
    return false;
  }

  if (!CostmapMediator::getPtrInstance()->isLayeredCostmapReady()) {
    LOG(ERROR) << "Is mediator's LayeredCostmap initialized? ";
    return false;
  }

  getParams();

  if (!b_enabled_) {
    LOG(ERROR) << s_name_ << " disabled";
    return false;
  }

  b_current_ = true;

  uc_default_value_ = toCost(0.5);

  layer_bound_.d_min_x = layer_bound_.d_min_y =
      std::numeric_limits<double>::max();
  layer_bound_.d_max_x = layer_bound_.d_max_y =
      -std::numeric_limits<double>::max();

  if (range_data_buffer_names_.empty()) {
    LOG(ERROR) << "range_data_buffer_names_.IsEmpty" << std::endl;
    return false;
  }

  for (const auto &topic_name : range_data_buffer_names_) {
    auto ptr_data = std::make_shared<CostmapRangeData>();
    CostmapMediator::getPtrInstance()
        ->registerUpdateFunc<std::shared_ptr<CostmapRangeData>>(topic_name,
                                                                ptr_data);

    // match with FIELD_NUM to avoid reallocate
    // memory when executing push_back.
    m_v_field_index_[topic_name].reserve(FIELD_NUM);
  }

  matchSize();
  return true;
}

void RangeSensorLayer::matchSize() {
  auto master = CostmapMediator::getPtrInstance()->getCostmap();

  double origin_x = master->getOriginX() + master->getSizeInMetersX() / 2 -
                    getSizeInMetersX() / 2;
  double origin_y = master->getOriginY() + master->getSizeInMetersY() / 2 -
                    getSizeInMetersY() / 2;

  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(),
            master->getResolution(), origin_x, origin_y);

  ptr_obs_time_.reset(
      new time_t[master->getSizeInCellsX() * master->getSizeInCellsY()]);
  time_t time_now = time(NULL);
  for (size_t i = 0; i < master->getSizeInCellsX() * master->getSizeInCellsY();
       i++) {
    ptr_obs_time_[i] = time_now;
  }
}

void RangeSensorLayer::getParams() {
  std::string sensor_type_name;
  CostmapMediator::getPtrInstance()->getParam(
      s_name_ + "." + "input_sensor_type", sensor_type_name,
      std::string("ALL"));

  CostmapMediator::getPtrInstance()->getParam(
      s_name_ + "." + "use_for_navigation", b_use_for_navigation_, true);

  d_resolution_ =
      CostmapMediator::getPtrInstance()->getCostmap()->getResolution();
  ui_size_x_ =
      CostmapMediator::getPtrInstance()->getCostmap()->getSizeInCellsX();
  ui_size_y_ =
      CostmapMediator::getPtrInstance()->getCostmap()->getSizeInCellsY();

  CostmapMediator::getPtrInstance()->getParam(s_name_ + "." + "phi", phi_v_,
                                              1.2);
  CostmapMediator::getPtrInstance()->getParam(s_name_ + "." + "inflate_cone",
                                              inflate_cone_, 1.0);
  CostmapMediator::getPtrInstance()->getParam(s_name_ + "." + "clear_threshold",
                                              clear_threshold_, 0.2);
  CostmapMediator::getPtrInstance()->getParam(s_name_ + "." + "mark_threshold",
                                              mark_threshold_, 0.8);
  CostmapMediator::getPtrInstance()->getParam(
      s_name_ + "." + "clear_on_max_reading", clear_on_max_reading_, true);

  CostmapMediator::getPtrInstance()->getParam(s_name_ + "." + "max_obs_range",
                                              max_obs_range_, 1.0);

  clear_cost_ = toCost(clear_threshold_);
  mark_cost_ = toCost(mark_threshold_);

  LOG(INFO) << "max_obs_range: " << max_obs_range_;

  CostmapMediator::getPtrInstance()->getParam(
      s_name_ + "." + "max_obs_keep_duration", max_obs_keep_duration_, 20.0);
  LOG(WARNING) << "max_obs_keep_duration " << max_obs_keep_duration_;

  if (sensor_type_name == "VARIABLE") {
    processRangeDataFunc_ =
        std::bind(&RangeSensorLayer::processVariableRangeMsg, this,
                  std::placeholders::_1);
  } else if (sensor_type_name == "FIXED") {
    processRangeDataFunc_ = std::bind(&RangeSensorLayer::processFixedRangeMsg,
                                      this, std::placeholders::_1);
  } else if (sensor_type_name == "ALL") {
    processRangeDataFunc_ = std::bind(&RangeSensorLayer::processRangeMsg, this,
                                      std::placeholders::_1);
  } else {
    LOG(INFO) << s_name_.c_str()
              << ": Invalid input sensor type: " << sensor_type_name.c_str()
              << std::endl;
  }
  /*===============================================================*/

  CostmapMediator::getPtrInstance()->getParam(s_name_ + "." + "enabled",
                                              b_enabled_, true);

  if (b_can_use_for_avoidance_) {
    CostmapMediator::getPtrInstance()->getParam(
        s_name_ + "." + "use_for_avoidance", b_use_for_avoidance_, true);
  }

  loadAvandanceBound();

  std::string topics_string;
  std::string source;
  range_data_buffer_names_.reserve(20);
  CostmapMediator::getPtrInstance()->getParam(
      s_name_ + ".topics", topics_string, std::string("sonar"));
  {
    std::stringstream ss(topics_string);
    while (ss >> source) { range_data_buffer_names_.push_back(source); }
  }
}

inline double RangeSensorLayer::gamma(const double &theta) {
  if (fabs(theta) > max_angle_) {
    return 0.0;
  } else {
    return 1 - pow(theta / max_angle_, 2);  // x^y
  }
}

inline double RangeSensorLayer::delta(const double &phi) {
  return 1 - (1 + tanh(2 * (phi - phi_v_))) / 2;
}

double RangeSensorLayer::sensorModel(const double &r, const double &phi,
                                     const double &theta) {
  if (r < d_resolution_)
    return 0.95;
  double lbda = delta(phi) * gamma(theta);

  double delta = d_resolution_;

  if (phi >= 0.0 && phi < r - 2 * delta * r) {
    return (1 - lbda) * (0.5);
  } else if (phi < r - delta * r) {
    return lbda * 0.5 * pow((phi - (r - 2 * delta * r)) / (delta * r), 2) +
           (1 - lbda) * .5;
  } else if (phi < r + delta * r) {
    double J = (r - phi) / (delta * r);
    return lbda * ((1 - (0.5) * pow(J, 2)) - 0.5) + 0.5;
  } else {
    return 0.5;
  }
}

void RangeSensorLayer::getDeltas(const double &angle, double &dx, double &dy) {
  double ta = tan(angle);
  if (ta == 0)
    dx = 0;
  else
    dx = d_resolution_ / ta;

  dx = copysign(dx, cos(angle));  //以 x 的模和 y 的符号组成浮点值
  dy = copysign(d_resolution_, sin(angle));
}

void RangeSensorLayer::updateCell(const WorldmapPose &wp_pose, const double &r,
                                  const WorldmapPoint &wp_point,
                                  const bool &clear) {
  static CostmapPoint point;
  if (worldToMap(wp_point, point)) {
    static double dx, dy, theta, phi, sensor, prior, prob_occ, prob_not,
        new_prob;

    dx = wp_point.d_x - wp_pose.d_x - 0.5 * d_resolution_;
    dy = wp_point.d_y - wp_pose.d_y - 0.5 * d_resolution_;
    theta = atan2(dy, dx) - wp_pose.d_yaw;
    theta = normalizeAngle(theta);
    phi = sqrt(dx * dx + dy * dy);

    // clear the origin
    if (phi <= d_resolution_) {
      theta = 0;
    }
    sensor = 0.0;
    // 概率更新模型
    // 若标志为true 当前存在障碍物更新概率为0
    if (!clear)
      sensor = sensorModel(r, phi, theta);

    // 获取当前点概率
    prior = toProb(getCost(point.ui_x, point.ui_y));
    if (prior > 0.6 && sensor == 0.5) {
      sensor = 0.45;
    }
    prob_occ = sensor * prior;
    prob_not = (1 - sensor) * (1 - prior);
    new_prob = prob_occ / (prob_occ + prob_not);

    static unsigned char c;
    c = toCost(new_prob);

    // 当是障碍物代价值，并且在设定的障碍物范围之外，则不更新cost
    // if (c >= mark_cost_ && r >= max_obs_range_) {
    //   return;
    // }

    // 更新代价值，和对应的代价值更新时间
    setCost(point.ui_x, point.ui_y, c);

    if (c >= mark_cost_) {
      size_t index = getIndex(point.ui_x, point.ui_y);
      if (index >= 0 && index < ui_size_x_ * ui_size_y_)
        ptr_obs_time_[index] = time(NULL);
    }
  }
}

bool RangeSensorLayer::updateSensorData() {
  for (auto it : range_data_buffer_names_) {
    std::string ids_;
    CostmapMediator::getPtrInstance()->getParam(std::string(it + ".frame_id"), ids_,std::string(""));
    std::stringstream id_ss_(ids_);
    std::string id_temp_;
    while (id_ss_ >> id_temp_){
      std::shared_ptr<CostmapRangeData> ptr_data = nullptr;
      if (!CostmapMediator::getPtrInstance()->getData(it+"."+id_temp_, ptr_data) ) continue;
      if (!processRangeDataFunc_(*ptr_data))
        LOG(ERROR) << s_name_ << ": process range data error." << std::endl;
    }
  }

  return true;
}

bool RangeSensorLayer::processRangeMsg(CostmapRangeData &range_data) {
  if (range_data.min_range_ == range_data.max_range_)
    return processFixedRangeMsg(range_data);
  else
    return processVariableRangeMsg(range_data);
}

bool RangeSensorLayer::processFixedRangeMsg(CostmapRangeData &range_data) {
  if (!isinf(range_data.range_)) {
    LOG(WARNING) << "Fixed distance ranger (min_range == max_range) in frame "
                    "%s sent invalid value. "
                    "Only -Inf (== object detected) and Inf (== no object "
                    "detected) are valid."
                 << s_name_ << std::endl;
    return false;
  }

  static bool clear_sensor_cone;
  clear_sensor_cone = false;

  if (range_data.range_ > 0) {
    if (!clear_on_max_reading_) {
      return false;
    }
    clear_sensor_cone = true;
  }

  range_data.range_ = range_data.min_range_;

  updateCostmap(range_data, clear_sensor_cone);

  return true;
}

bool RangeSensorLayer::processVariableRangeMsg(CostmapRangeData &range_data) {
  if (range_data.range_ <= range_data.min_range_) {
    return true;
  }
  bool clear_sensor_cone = false;
  double max_range_ = 0.0;
  CostmapMediator::getPtrInstance()->getParam(std::string(range_data.s_topic_name_ + "."+range_data.s_fram_id_+".max_d"), max_range_,0.0);
  if (range_data.range_ >= max_range_){
       range_data.range_ = range_data.max_range_;
       return true;
  }
  //预处理超声数据----------------------------------
  // 设置清楚标志 (最大值时 清楚)
  if ((fabs(range_data.range_ - range_data.max_range_) < 0.001) && clear_on_max_reading_) {
    clear_sensor_cone = true;
    //return true;
  }
  // 更新costmap
  updateCostmap(range_data, clear_sensor_cone);

  return true;
}

bool RangeSensorLayer::handleRangeData(
    const std::shared_ptr<CostmapRangeData> &ptr_range_data,
    std::vector<bool> &obstacle_area) {
  max_angle_ = ptr_range_data->field_of_view_ / 2;

  WorldmapPose wp_sensor_pose;
  wp_sensor_pose = ptr_range_data->sensor_pose_;

  // get left side point of sonar cone
  WorldmapPoint l_wp_point;
  l_wp_point.d_x = wp_sensor_pose.d_x +
                   cos(normalizeAngle(wp_sensor_pose.d_yaw + max_angle_)) *
                       ptr_range_data->range_;
  l_wp_point.d_y = wp_sensor_pose.d_y +
                   sin(normalizeAngle(wp_sensor_pose.d_yaw + max_angle_)) *
                       ptr_range_data->range_;

  // get right side point of sonar cone
  WorldmapPoint r_wp_point;
  r_wp_point.d_x = wp_sensor_pose.d_x +
                   cos(normalizeAngle(wp_sensor_pose.d_yaw - max_angle_)) *
                       ptr_range_data->range_;
  r_wp_point.d_y = wp_sensor_pose.d_y +
                   sin(normalizeAngle(wp_sensor_pose.d_yaw - max_angle_)) *
                       ptr_range_data->range_;

  // get middle side point of sonar cone
  WorldmapPoint m_wp_point;
  m_wp_point.d_x =
      wp_sensor_pose.d_x + cos(wp_sensor_pose.d_yaw) * ptr_range_data->range_;
  m_wp_point.d_y =
      wp_sensor_pose.d_y + sin(wp_sensor_pose.d_yaw) * ptr_range_data->range_;

  return true;
}

// 进入的接口
bool RangeSensorLayer::updateBounds(const WorldmapPose &wp_robot_pose,
                                    CostmapBound &cb_costmap_bound) {
  if (CostmapMediator::getPtrInstance()->isRolling()) {
    updateOrigin(wp_robot_pose.d_x - getSizeInMetersX() / 2,
                 wp_robot_pose.d_y - getSizeInMetersY() / 2);
  }

  // 根据时间阈值滤除旧的障碍物信息
  time_t time_now = time(NULL);
  for (size_t i = 0; i < ui_size_x_ * ui_size_y_; i++) {
    if (ptr_uc_costmap_[i] >= mark_cost_) {
      if (time_now - ptr_obs_time_[i] > max_obs_keep_duration_) {
        ptr_uc_costmap_[i] = getDefaultValue();
      }
    }
  }

  // 根据当前超声波信息更新传感器模型
  updateSensorData();

  cb_costmap_bound.d_min_x =
      std::min(cb_costmap_bound.d_min_x, layer_bound_.d_min_x);
  cb_costmap_bound.d_min_y =
      std::min(cb_costmap_bound.d_min_y, layer_bound_.d_min_y);
  cb_costmap_bound.d_max_x =
      std::max(cb_costmap_bound.d_max_x, layer_bound_.d_max_x);
  cb_costmap_bound.d_max_y =
      std::max(cb_costmap_bound.d_max_y, layer_bound_.d_max_y);

  layer_bound_.d_min_x = layer_bound_.d_min_y =
      std::numeric_limits<double>::max();
  layer_bound_.d_max_x = layer_bound_.d_max_y =
      std::numeric_limits<double>::min();

  if (!b_enabled_) {
    return false;
  }
  return true;
}

void RangeSensorLayer::updateCostmap(const CostmapRangeData &range_data,
                                     bool clear_sensor_cone) {
  // TODO: range_data.field_of_view_ = 0;
  max_angle_ = range_data.field_of_view_ / 2;

  // 变换外参 (将传感器变换至世界坐标)
  static WorldmapPose wp_sensor_pose;
  wp_sensor_pose = range_data.sensor_pose_;
  wp_sensor_pose.d_x =
      cos(range_data.world_pose_.d_yaw) * range_data.sensor_pose_.d_x -
      sin(range_data.world_pose_.d_yaw) * range_data.sensor_pose_.d_y +
      range_data.world_pose_.d_x;
  wp_sensor_pose.d_y =
      sin(range_data.world_pose_.d_yaw) * range_data.sensor_pose_.d_x +
      cos(range_data.world_pose_.d_yaw) * range_data.sensor_pose_.d_y +
      range_data.world_pose_.d_y;
  wp_sensor_pose.d_yaw = normalizeAngle(range_data.world_pose_.d_yaw +
                                        range_data.sensor_pose_.d_yaw);

  // 将当前障碍物变换至世界坐标
  static WorldmapPoint wp_range_point;
  wp_range_point.d_x =
      wp_sensor_pose.d_x + range_data.range_ * cos(wp_sensor_pose.d_yaw);
  wp_range_point.d_y =
      wp_sensor_pose.d_y + range_data.range_ * sin(wp_sensor_pose.d_yaw);

  static int bx0, by0, bx1, by1;
  static int Ox, Oy;

  // 确定机器人位姿是否在地图内部，做这种判断的主要原因是定位使用地图
  // 和导航使用地图在地图切换的瞬间可能不一致
  WorldmapPoint pose_point(wp_sensor_pose.d_x, wp_sensor_pose.d_y);
  CostmapPoint pose_pix;

  // 检查当前传感器坐标
  if (!worldToMap(pose_point, pose_pix)) {
    LOG(WARNING) << "robot pose out local map, cant update !";
    return;
  }
  Ox = pose_pix.ui_x;
  Oy = pose_pix.ui_y;

  bx1 = bx0 = Ox;
  by1 = by0 = Oy;

  // Update Map with Target Point
  // 此处对添加的障碍物点进行了距离限制
  static CostmapPoint cp_range_point;
  // 将障碍物转化至地图坐标
  if (worldToMap(wp_range_point, cp_range_point)) {
    // && range_data.range_ <= max_obs_range_) {
    // 设置障碍物代价值
    setCost(cp_range_point.ui_x, cp_range_point.ui_y, 233);
    touch(wp_range_point, layer_bound_);

    size_t index = getIndex(cp_range_point.ui_x, cp_range_point.ui_y);
    if (index >= 0 && index < ui_size_x_ * ui_size_y_)
      ptr_obs_time_[index] = time(NULL);
  }

  static WorldmapPoint wp_mpoint;
  static int Ax, Ay, Bx, By, Cx, Cy;

  // 填充fov内的点
  // Update left side of sonar cone
  wp_mpoint.d_x = wp_sensor_pose.d_x + cos(wp_sensor_pose.d_yaw - max_angle_) *
                                           range_data.range_ * 1.2;
  wp_mpoint.d_y = wp_sensor_pose.d_y + sin(wp_sensor_pose.d_yaw - max_angle_) *
                                           range_data.range_ * 1.2;
  worldToMapNoBounds(wp_mpoint.d_x, wp_mpoint.d_y, Ax, Ay);
  bx0 = std::min(bx0, Ax);
  bx1 = std::max(bx1, Ax);
  by0 = std::min(by0, Ay);
  by1 = std::max(by1, Ay);
  touch(wp_mpoint, layer_bound_);

  // Update right side of sonar cone
  wp_mpoint.d_x = wp_sensor_pose.d_x + cos(wp_sensor_pose.d_yaw + max_angle_) *
                                           range_data.range_ * 1.2;
  wp_mpoint.d_y = wp_sensor_pose.d_y + sin(wp_sensor_pose.d_yaw + max_angle_) *
                                           range_data.range_ * 1.2;
  worldToMapNoBounds(wp_mpoint.d_x, wp_mpoint.d_y, Bx, By);
  bx0 = std::min(bx0, Bx);
  bx1 = std::max(bx1, Bx);
  by0 = std::min(by0, By);
  by1 = std::max(by1, By);

  // Update far side of sonar cone
  worldToMapNoBounds(wp_range_point.d_x, wp_range_point.d_y, Cx, Cy);
  bx0 = std::min(bx0, Cx);
  bx1 = std::max(bx1, Cx);
  by0 = std::min(by0, Cy);
  by1 = std::max(by1, Cy);

  // Limit Bounds to Grid
  bx0 = std::max(0, bx0);
  by0 = std::max(0, by0);
  bx1 = std::min(static_cast<int>(ui_size_x_), bx1);
  by1 = std::min(static_cast<int>(ui_size_y_), by1);

  static bool update_xy_cell;
  static float bcciath;
  static int w0, w1, w2;

  if (cp_range_point.ui_x < bx0 || cp_range_point.ui_x > bx1 ||
      cp_range_point.ui_y < by0 || cp_range_point.ui_y > by1) {
    LOG(WARNING) << " cp_range_point x " << cp_range_point.ui_x << " y "
                 << cp_range_point.ui_y;
    LOG(WARNING) << "bx0->" << bx0 << " bx1->" << bx1 << " by0->" << by0
                 << " by1->" << by1;
    LOG(WARNING) << "Ax->" << Ax << " Ay->" << Ay << "   Bx->" << Bx << " By->"
                 << By << "   Cx->" << Cx << " Cy->" << Cy;
  }

  for (unsigned int x = bx0; x <= static_cast<unsigned int>(bx1); ++x) {
    for (unsigned int y = by0; y <= static_cast<unsigned int>(by1); ++y) {
      update_xy_cell = true;

      if (inflate_cone_ < 1.0) {
        // Determine barycentric coordinates， 质心坐标
        w0 = orient2d(Ax, Ay, Bx, By, x, y);
        w1 = orient2d(Bx, By, Ox, Oy, x, y);
        w2 = orient2d(Ox, Oy, Ax, Ay, x, y);

        bcciath = -inflate_cone_ * area(Ax, Ay, Bx, By, Ox, Oy);
        update_xy_cell = w0 >= bcciath && w1 >= bcciath && w2 >= bcciath;
      }

      if (update_xy_cell) {
        static WorldmapPoint wp_point;

        static CostmapPoint cp_point;
        cp_point.ui_x = x;
        cp_point.ui_y = y;
        mapToWorld(cp_point, wp_point);
        updateCell(wp_sensor_pose, range_data.range_, wp_point,
                   clear_sensor_cone);
      }
    }
  }
}

bool RangeSensorLayer::updateCosts(const std::shared_ptr<Costmap2d> master_grid,
                                   const int &i_min_i, const int &i_min_j,
                                   const int &i_max_i, const int &i_max_j) {
  if (!b_enabled_) {
    return false;
  }
  auto master_array = master_grid->getCharMap();
  static unsigned int span;
  span = master_grid->getSizeInCellsX();

  static unsigned int it;
  for (int j = i_min_j; j < i_max_j; j++) {
    it = j * span + i_min_i;
    for (int i = i_min_i; i < i_max_i; i++) {
      unsigned char prob = ptr_uc_costmap_[it];
      unsigned char current;
      if (prob == CVTE_BABOT::NO_INFORMATION) {
        it++;
        continue;
      } else if (prob >= mark_cost_) {
        current = CVTE_BABOT::LETHAL_OBSTACLE;
      } else if (prob <= clear_cost_) {
        current = CVTE_BABOT::FREE_SPACE;
      } else {
        it++;
        continue;
      }
      unsigned char old_cost = master_array[it];
      if (old_cost == NO_INFORMATION) {}
      if (old_cost < current) {
        master_array[it] = current;
      }
      it++;
    }
  }

  return true;
}

void RangeSensorLayer::updateOrigin(const double &d_new_origin_x,
                                    const double &d_new_origin_y) {
  int cell_ox =
      static_cast<int>((d_new_origin_x - d_origin_x_) / d_resolution_);
  int cell_oy =
      static_cast<int>((d_new_origin_y - d_origin_y_) / d_resolution_);

  double new_grid_ox = d_origin_x_ + cell_ox * d_resolution_;
  double new_grid_oy = d_origin_y_ + cell_oy * d_resolution_;

  int size_x = ui_size_x_;
  int size_y = ui_size_y_;

  // 重算当前原点在栅格地图上的坐标
  int lower_left_x = std::min(std::max(cell_ox, 0), size_x);
  int lower_left_y = std::min(std::max(cell_oy, 0), size_y);
  int upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
  int upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

  unsigned int cell_size_x = upper_right_x - lower_left_x;
  unsigned int cell_size_y = upper_right_y - lower_left_y;

  boost::shared_array<unsigned char> local_map =
      boost::shared_array<unsigned char>(
          new unsigned char[cell_size_x * cell_size_y]);

  boost::shared_array<time_t> ptr_local_obs_time =
      boost::shared_array<time_t>(new time_t[cell_size_x * cell_size_y]);

  // 缓存原 地图 && 时间地图
  copyMapRegion(ptr_uc_costmap_, lower_left_x, lower_left_y, ui_size_x_,
                local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);

  copyMapRegion(ptr_obs_time_, lower_left_x, lower_left_y, ui_size_x_,
                ptr_local_obs_time, 0, 0, cell_size_x, cell_size_x,
                cell_size_y);

  //重置
  resetMaps();

  d_origin_x_ = new_grid_ox;
  d_origin_y_ = new_grid_oy;

  // 偏移原点
  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;

  // 将之前的地图移动到新的位置
  copyMapRegion(local_map, 0, 0, cell_size_x, ptr_uc_costmap_, start_x, start_y,
                ui_size_x_, cell_size_x, cell_size_y);

  copyMapRegion(ptr_local_obs_time, 0, 0, cell_size_x, ptr_obs_time_, start_x,
                start_y, ui_size_x_, cell_size_x, cell_size_y);
}

void RangeSensorLayer::activate() {}
void RangeSensorLayer::deactivate() {}
void RangeSensorLayer::reset() {}

}  // namespace CVTE_BABOT
