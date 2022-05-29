/*
 * @Author: your name
 * @Date: 2021-06-01 16:43:20
 * @LastEditTime: 2021-07-15 15:23:22
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /navigation/navigation/speed_decision/src/speed_decision_base.cpp
 */

#include <limits.h>
#include "speed_decision_factory.hpp"
#include "speed_decision_base.hpp"
#include "glog/logging.h"
#include "robot_info.hpp"
#include "pnc_map.hpp"

#define LOG_HZ 20

namespace CVTE_BABOT {

std::shared_ptr<SpeedDecisionBaseConfig>
    SpeedDecision_Factor::speed_base_config_ = nullptr;

double SpeedDecision_Factor::controller_frequency_ = 1.0;

std::shared_ptr<SpeedDecisionBase> SpeedDecisionBase::ptr_speed_decision_ =
    nullptr;

SpeedDecisionBase::SpeedDecisionBase() {
  LOG(INFO) << "construct speed decision";
  mission_manager_speed_radio_ = SpeedRadio::LEVEL4;
  localization_speed_radio_ = SpeedRadio::LEVEL4;
  mark_speed_radio_ = SpeedRadio::LEVEL4;
  stop_bound_speed_radio_ = SpeedRadio::LEVEL4;

  speed_radio2double_[SpeedRadio::STOP] = 0.0;
  speed_radio2double_[SpeedRadio::LEVEL1] = 0.25;
  speed_radio2double_[SpeedRadio::LEVEL2] = 0.5;
  speed_radio2double_[SpeedRadio::LEVEL3] = 0.75;
  speed_radio2double_[SpeedRadio::LEVEL4] = 1.0;

  last_vel_.d_x = 0;
  last_vel_.d_y = 0;
  last_vel_.d_yaw = 0;
  ptr_costmap_2d_ = nullptr;
  config_flag_ = false;

  ptr_marker_map_.reset(new MarkerMap());

  m_recover_coverage_.insert(std::make_pair(RecoveDir::RECOVER_FRONT, "front"));
  m_recover_coverage_.insert(std::make_pair(RecoveDir::RECOVER_BACK, "back"));
  m_recover_coverage_.insert(std::make_pair(RecoveDir::RECOVER_LEFT, "left"));
  m_recover_coverage_.insert(std::make_pair(RecoveDir::RECOVER_RIGHT, "right"));
}

void SpeedDecisionBase::setInstance(
    const std::shared_ptr<SpeedDecisionBase> &ptr) {
  ptr_speed_decision_ = ptr;
};

std::shared_ptr<SpeedDecisionBase> SpeedDecisionBase::getInstance() {
  if (ptr_speed_decision_ == nullptr) {
    LOG(ERROR) << "speed_decision is not creat";
    return nullptr;
  } else {
    return ptr_speed_decision_;
  }
}

void SpeedDecisionBase::setConfig() {
  // check ptr
  if (ptr_config_ == nullptr) {
    LOG(ERROR) << "ptr_config_ is nullptr ";
    return;
  }

  // check and set shape
  // stop
  if (ptr_config_->m_shape.find("stop") == ptr_config_->m_shape.end()) {
    LOG(ERROR) << "No stop_shape are set";
  } else {
    LOG(INFO) << "begin set stop shape ";
    setRobotShapeConfig(stop_shape_, ptr_config_->m_shape["stop"],
                        ptr_config_->costmap_traverse_inc);
  }

  // slow
  if (ptr_config_->m_shape.find("slow") == ptr_config_->m_shape.end()) {
    LOG(ERROR) << "No slow_shape are set";
  } else {
    LOG(INFO) << "begin set slow shape ";
    setRobotShapeConfig(slow_shape_, ptr_config_->m_shape["slow"],
                        ptr_config_->costmap_traverse_inc);
  }

  // recover
  if (ptr_config_->m_shape.find("recover") == ptr_config_->m_shape.end()) {
    LOG(ERROR) << "No recover_shape are set";

  } else {
    LOG(INFO) << "begin set recover shape ";
    setRobotShapeConfig(recover_shape_, ptr_config_->m_shape["recover"],
                        ptr_config_->costmap_traverse_inc);
  }

  // danger
  if (ptr_config_->m_shape.find("danger") == ptr_config_->m_shape.end()) {
    LOG(ERROR) << "No danger_shape are set";

  } else {
    LOG(INFO) << "begin set danger shape ";
    setRobotShapeConfig(danger_shape_, ptr_config_->m_shape["danger"],
                        ptr_config_->costmap_traverse_inc);
  }

  // arc_recover
  if (ptr_config_->m_shape.find("arc_recover") == ptr_config_->m_shape.end()) {
    LOG(ERROR) << "No arc_recover are set";
  } else {
    LOG(INFO) << "begin set danger shape ";
    setRobotShapeConfig(arc_recover_shape_, ptr_config_->m_shape["arc_recover"],
                        ptr_config_->costmap_traverse_inc);
  }
  config_flag_ = true;
}

void SpeedDecisionBase::setRobotShapeConfig(
    StopShape &address, const std::vector<StopShapeConfig> &v_shape,
    double resolution) {
  address.setResolution(resolution);

  for (auto &shape : v_shape) {
    address.push(shape.key, shape.params, shape.start_theta, shape.end_theta);
  }
}

void SpeedDecisionBase::inputScan(const std::vector<Eigen::Vector2d> &scan) {
  std::lock_guard<std::mutex> lock_scan(scan_mutex_);
  scan_ = scan;
  for (size_t index = 0; index < scan_.size(); index++) {
    scan_[index] = ptr_config_->T_bl.block<2, 2>(0, 0) * scan_[index] +
                   ptr_config_->T_bl.block<2, 1>(0, 2);
  }
  updateScanTime();
}

void SpeedDecisionBase::inputAttitude(
    const Eigen::Matrix<double, 3, 3> &attitude) {
  std::lock_guard<std::mutex> lock_att(attitude_mutex_);
  robot_attitude_ = attitude;
  updateAttitudeTime();
}

void SpeedDecisionBase::inputSonar(
    const std::vector<Eigen::Vector2d> &range_points) {
  std::lock_guard<std::mutex> lock_sonar(sonar_data_mutex_);
  sonar_sensor_data_ = range_points;
}
void SpeedDecisionBase::inputInfrared(
    const std::vector<Eigen::Vector2d> &range_points) {
  std::lock_guard<std::mutex> lock_infrared(ir_data_mutex_);
  ir_sensor_data_ = range_points;
}

void SpeedDecisionBase::setMissionManagerSpeedValue(double value) {
  ptr_config_->max_v = value;
  mission_manager_speed_radio_ = SpeedRadio::LEVEL4;
  LOG(INFO) << "set max velocity " << ptr_config_->max_v;
}

void SpeedDecisionBase::setMissionMangerStripMark(
    const Json::Value &json_file) {
  if (!ptr_global_costmap_2d_) {
    LOG(WARNING) << "no gloabl costmap in speed decision object !";
    return;
  }

  // 更新marker地图参数
  if (!ptr_marker_map_) {
    ptr_marker_map_.reset(new MarkerMap());
  }
  ptr_marker_map_->resetMap(ptr_global_costmap_2d_);

  // 设置减速带区域
  ptr_marker_map_->setStripsMarker(json_file);
}

void SpeedDecisionBase::setMissionManagerSpeedRadio(
    const SpeedRadio &speed_radio) {
  LOG(INFO) << "mission spd_radio is " << speed_radio;
  mission_manager_speed_radio_ = speed_radio;
}

void SpeedDecisionBase::setLocalizationSpeedRadio(
    const SpeedRadio &speed_radio) {
  LOG(INFO) << "Localization spd_radio is " << speed_radio;
  localization_speed_radio_ = speed_radio;
}

void SpeedDecisionBase::setMarkSpeedRadio(const SpeedRadio &speed_radio) {
  LOG(INFO) << "Mark spd_radio is " << speed_radio;
  mark_speed_radio_ = speed_radio;
}

SpeedDecisionResult SpeedDecisionBase::getSpeedResult() {
  // 获取速度决策的结果
  std::lock_guard<std::mutex> lock_result(speed_deci_result_mutex_);
  return speed_deci_result_;
}

Velocity SpeedDecisionBase::getActualVel(const Velocity &controller_vel) {
  // 速度获取接口
  speed_deci_result_mutex_.lock();

  // 速度限幅
  double limit_v = speed_deci_result_.max_v;
  double limit_w = speed_deci_result_.max_w;
  SpeedRadio speed_radio = speed_deci_result_.speed_radio;
  speed_deci_result_mutex_.unlock();
  Velocity ret;
  if (speed_radio == SpeedRadio::STOP) {
    ret.d_x = 0;
    ret.d_y = 0;
    ret.d_yaw = 0;

    //清空上次保存的速度
    last_vel_.d_x = 0;
    last_vel_.d_y = 0;
    last_vel_.d_yaw = 0;
    return ret;
  }

  // 平滑当前的速度
  double factor_v = 1.0;
  double factor_w = 1.0;
  if (fabs(controller_vel.d_x) > limit_v) {
    factor_v = limit_v / fabs(controller_vel.d_x);
  }
  ret.d_x = factor_v * controller_vel.d_x;
  ret.d_yaw = factor_v * controller_vel.d_yaw;
  //在电梯内最大角速度为0.5
  Pose2d cur_pose = RobotInfo::getPtrInstance()->getCurrentPose();
  PncMap pnc_map;
  if (pnc_map.InElevatorArea(
          math_utils::Vec2d(cur_pose.getX(), cur_pose.getY()))) {
    limit_w = 0.5;
  }
  if (fabs(ret.d_yaw) > limit_w) {
    factor_w = limit_w / fabs(ret.d_yaw);
  }
  ret.d_x = factor_w * ret.d_x;
  ret.d_yaw = factor_w * ret.d_yaw;
  Velocity smooth_vel = smoothVel(ret);
  checkMinVel(smooth_vel);

  LOG_EVERY_N(INFO, LOG_HZ)
      << "controller_vel: " << controller_vel.d_x << " " << controller_vel.d_y
      << " " << controller_vel.d_yaw;
  LOG_EVERY_N(INFO, LOG_HZ) << "ret_vel: " << ret.d_x << " " << ret.d_yaw;
  LOG_EVERY_N(INFO, LOG_HZ)
      << "limit_vel: " << limit_v << " " << limit_w << " ";
  LOG_EVERY_N(INFO, LOG_HZ)
      << "smooth_vel: " << smooth_vel.d_x << " " << smooth_vel.d_yaw;

  return smooth_vel;
}

void SpeedDecisionBase::setReferIndex(size_t refer_index) {
  refer_index_ = refer_index;
}

void SpeedDecisionBase::setReferPath(std::shared_ptr<SubPath> refer_path) {
  std::lock_guard<std::mutex> lock_path(path_mutex_);
  refer_path_ = refer_path;
}

void SpeedDecisionBase::setCostmap(std::shared_ptr<Costmap2d> ptr_costmap_2d) {
  // LOG(INFO) << "speed decision set costmap";
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  ptr_costmap_2d_ = ptr_costmap_2d;
  // if (ptr_costmap_2d_ == nullptr) {
  //   LOG(INFO) << "costmap still empty";
  // }
}

void SpeedDecisionBase::setGlobalCostmap(
    std::shared_ptr<Costmap2d> ptr_costmap_2d) {
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  ptr_global_costmap_2d_ = ptr_costmap_2d;
}

//主线程
void SpeedDecisionBase::updateVel() {
  auto scan_time_diff = calcuScanTimeDiff();
  auto attitude_time_diff = calcuAttitudeTimeDiff();

  // 获取速度因子
  double speed_factor = getCurSpeedFactor();  // 停障 与 减速判断

  std::lock_guard<std::mutex> lock(speed_deci_result_mutex_);

  if (!config_flag_) {
    LOG_EVERY_N(INFO, LOG_HZ) << "not set SpeedDecisionBase config";
    stop_bound_speed_radio_ = SpeedRadio::STOP;
    speed_deci_result_.max_v = 0;
    speed_deci_result_.max_w = 0;
    speed_deci_result_.real_speed_controller = "SpeedController";
    speed_deci_result_.speed_radio = SpeedRadio::STOP;
    speed_deci_result_.double_speed_radio = 0;
    return;
  }
  if (speed_factor < 0.01) {
    // 进入传感器停障部分
    stop_bound_speed_radio_ = SpeedRadio::STOP;
    speed_deci_result_.max_v = 0;
    speed_deci_result_.max_w = 0;
    speed_deci_result_.real_speed_controller = "SpeedController";
    speed_deci_result_.speed_radio = SpeedRadio::STOP;
    speed_deci_result_.double_speed_radio = 0;
    return;
  }

  // 其他模块传入的停障部分
  if (mark_speed_radio_ == SpeedRadio::STOP) {
    speed_deci_result_.max_v = 0;
    speed_deci_result_.max_w = 0;
    speed_deci_result_.real_speed_controller = "Mark";
    speed_deci_result_.speed_radio = SpeedRadio::STOP;
    // speed_deci_result_.double_speed_radio = 0;
    return;
  }
  if (localization_speed_radio_ == SpeedRadio::STOP) {
    speed_deci_result_.max_v = 0;
    speed_deci_result_.max_w = 0;
    speed_deci_result_.real_speed_controller = "Localization";
    speed_deci_result_.speed_radio = SpeedRadio::STOP;
    // speed_deci_result_.double_speed_radio = 0;
    return;
  }
  if (mission_manager_speed_radio_ == SpeedRadio::STOP) {
    speed_deci_result_.max_v = 0;
    speed_deci_result_.max_w = 0;
    speed_deci_result_.real_speed_controller = "Mission_manager";
    speed_deci_result_.speed_radio = SpeedRadio::STOP;
    // speed_deci_result_.double_speed_radio = 0;
    return;
  }

  // 速度等级
  stop_bound_speed_radio_ = SpeedRadio::LEVEL4;
  SpeedRadio speed_radio =
      std::min({mission_manager_speed_radio_, localization_speed_radio_,
                mark_speed_radio_},
               [&](SpeedRadio a, SpeedRadio b) {
                 return (speed_radio2double_[a] < speed_radio2double_[b]);
               });

  double cur_speed_radio = speed_radio2double_[speed_radio];

  // 计算当前最大速度
  // 当前速度等级 * 减速因子计算出的最大速度
  speed_deci_result_.max_v =
      cur_speed_radio * (speed_factor * ptr_config_->max_v);
  // speed_deci_result_.max_w =
  //     cur_speed_radio * (speed_factor * ptr_config_->max_w);

  speed_deci_result_.max_w = ptr_config_->max_w;
  speed_deci_result_.real_speed_controller = "SpeedController";
  speed_deci_result_.speed_radio = speed_radio;
  speed_deci_result_.double_speed_radio = speed_factor;
}

double SpeedDecisionBase::getMissionManagerSpeedValue() {
  if (fabs(ptr_config_->max_v) <= std::numeric_limits<double>::epsilon() ||
      ptr_config_->max_v > 1.0) {
    return 1.0;
  }
  return ptr_config_->max_v;
}

std::chrono::duration<double> SpeedDecisionBase::calcuScanTimeDiff() {
  auto now = std::chrono::system_clock::now();
  scan_time_mutex_.lock();
  std::chrono::duration<double> diff = now - last_scan_time_;
  scan_time_mutex_.unlock();
  return diff;
}

std::chrono::duration<double> SpeedDecisionBase::calcuAttitudeTimeDiff() {
  auto now = std::chrono::system_clock::now();
  attitude_time_mutex_.lock();
  std::chrono::duration<double> diff = now - last_attitude_time_;
  attitude_time_mutex_.unlock();
  return diff;
}

void SpeedDecisionBase::updateScanTime() {
  scan_time_mutex_.lock();
  last_scan_time_ = std::chrono::system_clock::now();
  scan_time_mutex_.unlock();
}

void SpeedDecisionBase::updateAttitudeTime() {
  std::lock_guard<std::mutex> lock_attitude_time(attitude_time_mutex_);
  last_attitude_time_ = std::chrono::system_clock::now();
}

// 速度因子部分
double SpeedDecisionBase::getCurSpeedFactor() {
  LOG(INFO) << "Speed Decision Callback";
  if (isObstacleInStopBound()) {
    LOG(ERROR) << "point in obstacle bound!!!!!!!";
    return 0;
  }
  double slow_bound_factor = getSlowBoundFactor();  // 预设的减速框 固定大小
  double path_speed_factor = getPathSlowFactor();  // 路径上机器与障碍物的距离
  double elevator_speed_factor = 1.0;
  if (RobotInfo::getPtrInstance()->getMissionType() == MISSIONTYPE::ELVATOR) {
    elevator_speed_factor = getElevatorFactor();
    path_speed_factor = 1.0;
    slow_bound_factor = 1.0;
  }
  double marker_factor = getMarkerFactor();  // 部署时设定的减速带
  double camera_factor = getCameraFactor();  // 相机点云判断的减速
  // double slope_factor = getSlopeFactor();    // 机器倾斜(斜坡上的减速)
  double slope_factor = 1.0;

  LOG_EVERY_N(INFO, LOG_HZ) << "path_speed_factor: " << path_speed_factor;
  LOG_EVERY_N(INFO, LOG_HZ) << "slow_bound_factor: " << slow_bound_factor;
  LOG_EVERY_N(INFO, LOG_HZ) << "marker factor: " << marker_factor;
  LOG_EVERY_N(INFO, LOG_HZ) << "camera factor: " << camera_factor;
  LOG_EVERY_N(INFO, LOG_HZ) << "slpoe factor: " << slope_factor;

  double min_factor =
      std::min({path_speed_factor, slow_bound_factor, marker_factor,
                camera_factor, slope_factor, elevator_speed_factor});

  return min_factor;
}

double SpeedDecisionBase::getSlowBoundFactor() {
  // get slow point;
  // slow_shape_.clearInc();
  // Velocity cur_vel = RobotInfo::getPtrInstance()->getCurrentVel();
  // double inc_x = 0.3 * cur_vel.d_x;
  // double inc_y = ptr_config_->driver_type * 0.2 * cur_vel.d_yaw;
  // if (inc_x > 0) {
  //   slow_shape_.setInc(SHAPE_FRONT, fabs(inc_x));
  // } else {
  //   slow_shape_.setInc(SHAPE_BACK, fabs(inc_x));
  // }
  // if (inc_y > 0) {
  //   slow_shape_.setInc(SHAPE_LEFT, fabs(inc_y));
  // } else {
  //   slow_shape_.setInc(SHAPE_RIGHT, fabs(inc_y));
  // }
  // double x_max = INT_MIN;
  // double x_min = INT_MAX;
  // double y_max = INT_MIN;
  // double y_min = INT_MAX;
  // slow_shape_.getBoundingbox(x_max, x_min, y_max, y_min);
  // auto cur_pose = RobotInfo::getPtrInstance()->getCurrentPose();

  // Eigen::Matrix<double, 2, 3> T_wb;
  // T_wb << cos(cur_pose.getYaw()), -sin(cur_pose.getYaw()), cur_pose.getX(),
  //     sin(cur_pose.getYaw()), cos(cur_pose.getYaw()), cur_pose.getY();

  // double factor = 1.0;
  // for (double x = x_min; x < x_max; x += ptr_config_->costmap_traverse_inc) {
  //   for (double y = y_min; y < y_max; y += ptr_config_->costmap_traverse_inc)
  //   {
  //     Eigen::Vector2d point_in_world =
  //         T_wb.block<2, 2>(0, 0) * Eigen::Vector2d(x, y) +
  //         T_wb.block<2, 1>(0, 2);
  //     if (checkPointCostValue(point_in_world) == 254) {
  //       //优化
  //       factor = 0.4;
  //       break;
  //     }
  //   }
  // }
  // return factor > 0.3 ? factor : 0.3;

  // 初始化变量
  bool finish_flag = false;
  double factor = 1.0;
  double traverse_inc = ptr_config_->costmap_traverse_inc;
  double x_max = INT_MIN;
  double x_min = INT_MAX;
  double y_max = INT_MIN;
  double y_min = INT_MAX;
  slow_shape_.clearInc();
  Velocity cur_vel = RobotInfo::getPtrInstance()->getCurrentVel();
  auto cur_pose = RobotInfo::getPtrInstance()->getCurrentPose();

  Velocity ues_vel = cur_vel;
  ues_vel.d_x =
      fabs(last_vel_.d_x) > fabs(cur_vel.d_x) ? last_vel_.d_x : cur_vel.d_x;
  ues_vel.d_yaw = fabs(last_vel_.d_yaw) > fabs(cur_vel.d_yaw) ? last_vel_.d_yaw
                                                              : cur_vel.d_yaw;

  Eigen::Matrix<double, 2, 3> T_wb;
  T_wb << cos(cur_pose.getYaw()), -sin(cur_pose.getYaw()), cur_pose.getX(),
      sin(cur_pose.getYaw()), cos(cur_pose.getYaw()), cur_pose.getY();

  // 原地旋转时  角速度为 20°/s && 线速度较小
  if (fabs(ues_vel.d_x) < 0.15 && fabs(ues_vel.d_yaw) > 0.15) {
    double min_point_y = INT_MAX;
    double origin_y = 0;
    LOG(INFO) << "slow_bound_factor: d_yaw " << ues_vel.d_yaw;
    if (ues_vel.d_yaw > 0) {
      // 查找最近点
      slow_shape_.getBoundingbox(x_max, x_min, y_max, y_min, "left");
      origin_y = y_min;
      for (double x = x_min; x < x_max; x += traverse_inc) {
        for (double y = y_min; y < y_max; y += traverse_inc) {
          Eigen::Vector2d point_in_world =
              T_wb.block<2, 2>(0, 0) * Eigen::Vector2d(x, y) +
              T_wb.block<2, 1>(0, 2);
          if (checkPointCostValue(point_in_world) == 254) {
            min_point_y = y;
            finish_flag = true;
            break;
          }
        }
        if (finish_flag) {
          break;
        }
      }
    } else {
      // 查找最近点
      slow_shape_.getBoundingbox(x_max, x_min, y_max, y_min, "right");
      origin_y = y_max;
      for (double x = x_min; x < x_max; x += traverse_inc) {
        for (double y = y_max; y > y_min; y -= traverse_inc) {
          Eigen::Vector2d point_in_world =
              T_wb.block<2, 2>(0, 0) * Eigen::Vector2d(x, y) +
              T_wb.block<2, 1>(0, 2);
          if (checkPointCostValue(point_in_world) == 254) {
            min_point_y = y;
            finish_flag = true;
            break;
          }
        }
        if (finish_flag) {
          break;
        }
      }
    }

    // 计算因子
    if (finish_flag == false) {
      return 1.0;
    }
    double obs_dis = fabs(min_point_y - origin_y);
    factor = (obs_dis / (fabs(y_max - y_min)));
    factor = factor < 0.3 ? 0.3 : factor;
    return factor;
  } else {
    double min_point_x = INT_MAX;
    double origin_x = 0;
    // 如果角速度较大 增加横向的增量
    if (fabs(ues_vel.d_x) > 0.3) {
      double inc_y = 0.2 * ues_vel.d_yaw;
      if (inc_y > 0) {
        // 查找最近点
        slow_shape_.setInc(SHAPE_LEFT, fabs(inc_y));
      } else {
        // 查找最近点
        slow_shape_.setInc(SHAPE_RIGHT, fabs(inc_y));
      }
    }

    if (ues_vel.d_x < 0) {
      slow_shape_.getBoundingbox(x_max, x_min, y_max, y_min, "back");
      origin_x = x_max;
      for (double y = y_min; y < y_max; y += traverse_inc) {
        for (double x = x_max; x > x_min; x -= traverse_inc) {
          Eigen::Vector2d point_in_world =
              T_wb.block<2, 2>(0, 0) * Eigen::Vector2d(x, y) +
              T_wb.block<2, 1>(0, 2);
          if (checkPointCostValue(point_in_world) == 254) {
            min_point_x = x;
            finish_flag = true;
            break;
          }
        }
        if (finish_flag) {
          break;
        }
      }
    } else {
      slow_shape_.getBoundingbox(x_max, x_min, y_max, y_min, "front");
      origin_x = x_min;
      for (double y = y_min; y < y_max; y += traverse_inc) {
        for (double x = x_min; x < x_max; x += traverse_inc) {
          Eigen::Vector2d point_in_world =
              T_wb.block<2, 2>(0, 0) * Eigen::Vector2d(x, y) +
              T_wb.block<2, 1>(0, 2);
          if (checkPointCostValue(point_in_world) == 254) {
            min_point_x = x;
            finish_flag = true;
            break;
          }
        }
        if (finish_flag) {
          break;
        }
      }
    }

    // 计算因子
    if (finish_flag == false) {
      return 1.0;
    }
    double obs_dis = fabs(min_point_x - origin_x);
    factor = (obs_dis / (fabs(x_max - x_min)));
    factor = factor < 0.4 ? 0.4 : factor;
    return factor;
  }
}

double SpeedDecisionBase::getPathSlowFactor() {
  if (refer_path_ == nullptr) {
    return 1.0;
  }
  //收水逻辑，路径长度小于2m时减速到0.1m
  // if (refer_path_->wpis.back().path_length -
  //             refer_path_->wpis[refer_index_].path_length <
  //         2.0 &&
  //     RobotInfo::getPtrInstance()->getActionType() == ActionType::CLEAN &&
  //     refer_path_->type == PathType::REFER) {
  //   return 0.1 / ptr_config_->max_v;
  // }
  double sum_dist = 0;
  size_t index = 1;
  std::lock_guard<std::mutex> lock(path_mutex_);
  double path_slow_factor;
  size_t skip = 3;
  for (index = refer_index_ + skip; index < refer_path_->wps.size();
       index += skip) {
    // 计算整条路径的长度
    sum_dist += refer_path_->wpis[index].path_length -
                refer_path_->wpis[index - skip].path_length;

    // 前方costmap 存在障碍 || 已超过搜索范围
    if (checkPointCostValue(Eigen::Vector2d(refer_path_->wps[index].getX(),
                                            refer_path_->wps[index].getY())) >
            ptr_config_->stop_cost ||                // stop_cost = 100.0
        sum_dist > ptr_config_->path_slow_length) {  // length = 3.0
      break;
    }
  }
  if (sum_dist < ptr_config_->path_slow_length &&
      index < refer_path_->wps.size()) {
    // 离障碍物越近(距离越短) 因子越小 配置文件
    path_slow_factor = sum_dist / ptr_config_->path_slow_length;
  } else {
    // 超过搜索范围
    path_slow_factor = 1.0;
  }

  // 限幅 最小输出0.2
  path_slow_factor = path_slow_factor > 0.2 ? path_slow_factor : 0.2;
  return path_slow_factor;
}

double SpeedDecisionBase::getMarkerFactor() {
  // TODO: 这个后面要删掉 语义地图
  return 1.0;
}

double SpeedDecisionBase::getCameraFactor() {
  std::vector<Pose2d> camera_cloud =
      RobotInfo::getPtrInstance()->getCameraCloud();

  int obs_cnt = 0;
  float distance_sum = 0;

  std::vector<double> v_point;
  for (int i = 0; i < camera_cloud.size(); i++) {
    if (camera_cloud[i].x < ptr_config_->max_camera_distance) {
      ++obs_cnt;
      v_point.push_back(camera_cloud[i].x);
    }
  }

  // 换算速度因子 防止噪点
  if (obs_cnt > 5) {
    std::sort(v_point.begin(), v_point.end());
    for (int i = 0; i < 5; ++i) { distance_sum += v_point[i]; }

    double distance = distance_sum / 5.0;
    double factor = (distance / ptr_config_->max_camera_distance);
    factor = factor < 0.3 ? 0.3 : factor;
    return factor;
  } else {
    return 1.0;
  }
}

double SpeedDecisionBase::getInfraredObstacleFactor() {
  double infraredFactor = 1.0;
  std::map<std::string, double> infrared_obstacle =
      RobotInfo::getPtrInstance()->getInfraredObstacleRange();
  for (int i = 0; i < infrared_obstacle.size(); i++) {
    std::string d = "infrared_obstacle_0";
    std::string dd = d + std::to_string(i);

    if (infrared_obstacle[dd] > 120.0 && infrared_obstacle[dd] <= 500) {
      return (-0.002 * infrared_obstacle[dd] +
              1.2526);  //线性递减（120 对应1.0， 500 对应0.2）
    }

    else {
      infraredFactor = 1.0;
    }
  }
  return infraredFactor;
}

double SpeedDecisionBase::getSlopeFactor() {
  std::lock_guard<std::mutex> lock(attitude_mutex_);
  Eigen::Vector3d ypr = rotation2rpy(robot_attitude_);

  double max_roll = std::max(fabs(ypr(0)), fabs(ypr(1)));
  // LOG_EVERY_N(INFO, LOG_HZ)
  //     << "robot attitude: " << ypr.transpose() << std::endl
  //     << " max_roll: " << max_roll;

  if (max_roll < ptr_config_->min_slope) {
    // 倾角小于 3°
    return 1.0;
  }
  if (max_roll > ptr_config_->max_slope) {
    // 倾角大于 15°
    return 0.3;
  }

  double factor = 0.7 * (max_roll - ptr_config_->min_slope) /
                  (ptr_config_->max_slope - ptr_config_->min_slope);
  return 1 - factor;
}

double SpeedDecisionBase::getElevatorFactor() {
  std::vector<Eigen::Vector2d> bounding_box = getBoundingbox("stop");
  Pose2d robot_pose = RobotInfo::getPtrInstance()->getCurrentPose();
  if (bounding_box.size() < 2) {
    return 0.2;
  }
  double front = bounding_box.front().x();
  double rear = bounding_box.front().x();
  for (const auto &it : bounding_box) {
    if (it.x() > front) {
      front = it.x();
    }
    if (it.x() < rear) {
      rear = it.x();
    }
  }
  Pose2d front_pt(front, 0.0, 0.0);
  Pose2d rear_pt(rear, 0.0, 0.0);
  LOG(INFO) << "front: " << front;
  LOG(INFO) << "rear: " << rear;
  Pose2d cur_front_pt = robot_pose * front_pt;
  Pose2d cur_rear_pt = robot_pose * rear_pt;
  PncMap pnc_map;
  bool front_in = false;
  bool rear_in = false;
  if (pnc_map.InElevatorArea(
          math_utils::Vec2d(cur_front_pt.getX(), cur_front_pt.getY()))) {
    front_in = true;
  }
  if (pnc_map.InElevatorArea(
          math_utils::Vec2d(cur_rear_pt.getX(), cur_rear_pt.getY()))) {
    rear_in = true;
  }
  if (rear_in && !front_in) {
    LOG(INFO) << "rear in elevator, front not in elevator: 0.5";
    return 0.5;
  }
  if (front_in && !rear_in) {
    LOG(INFO) << "front in elevator, rear not in elevator: 0.5";
    return 0.5;
  }
  LOG(INFO) << "front nor rear in elevator: 0.3";
  return 0.3;
}

bool SpeedDecisionBase::isObstacleInStopBound() {
  Velocity cur_vel = RobotInfo::getPtrInstance()->getCurrentVel();
  // 清空停障框增量
  stop_shape_.clearInc();

  // 获取贴边标志
  getEdgeFlag();

  // 根据任务调整停障框
  if (edge_flag_ ||
      RobotInfo::getPtrInstance()->getMissionType() == MISSIONTYPE::ELVATOR) {
    // if (edge_flag_) {
    //   // TODO: 改成配置文件
    //   if (refer_path_->wpis[refer_index_].edge_dist > 0) {
    //     stop_shape_.setInc(SHAPE_LEFT, -0.02);
    //   } else {
    //     stop_shape_.setInc(SHAPE_RIGHT, -0.02);
    //   }
    // }
    // else if (RobotInfo::getPtrInstance()->getMissionType() ==
    //            MISSIONTYPE::ELVATOR) {
    //   stop_shape_.setInc(SHAPE_LEFT, -0.05);
    //   stop_shape_.setInc(SHAPE_RIGHT, -0.05);
    // }
    // TODO: 改成配置文件
    // stop_shape_.setInc(SHAPE_FRONT, -0.05);
  } else {
    double inc_x = ptr_config_->extra_stop_v_radio * cur_vel.d_x;
    double inc_y = ptr_config_->driver_type * ptr_config_->extra_stop_w_radio *
                   cur_vel.d_yaw;

    // if (inc_x > 0) {
    //   stop_shape_.setInc(SHAPE_FRONT, fabs(inc_x));
    // } else {
    //   stop_shape_.setInc(SHAPE_BACK, fabs(inc_x));
    // }
    if (inc_y > 0) {
      stop_shape_.setInc(SHAPE_LEFT, fabs(inc_y));
    } else {
      stop_shape_.setInc(SHAPE_RIGHT, fabs(inc_y));
    }
  }

  bool obstacle_stop = false;
  std::vector<Eigen::Vector2d> show_crash_point;

  // 雷达停障
  std::lock_guard<std::mutex> lock_scan(scan_mutex_);
  if (stop_shape_.checkPonitInside(scan_, "all", &show_crash_point)) {
    LOG(WARNING) << "scan stop x " << show_crash_point[0](0) << " y "
                 << show_crash_point[0](1);
    return true;
  }

  // 超声模块
  // std::lock_guard<std::mutex> lock_sonar(sonar_data_mutex_);
  // if (stop_shape_.checkPonitInside(sonar_sensor_data_)) {
  //   LOG(WARNING) << "sonar stop";
  //   return true;
  // }

  // 红外模块
  // std::lock_guard<std::mutex> lock_ir(ir_data_mutex_);
  // if (stop_shape_.checkPonitInside(ir_sensor_data_)) {
  //   LOG(WARNING) << "ir stop";
  //   return true;
  // }

  // 代价地图
  if (RobotInfo::getPtrInstance()->getMissionType() == MISSIONTYPE::ELVATOR) {
    stop_shape_.setInc(SHAPE_LEFT, -0.2);
    stop_shape_.setInc(SHAPE_RIGHT, -0.2);
    stop_shape_.setInc(SHAPE_FRONT, -0.05);
  } else {
    stop_shape_.setInc(SHAPE_LEFT, -0.08);
    stop_shape_.setInc(SHAPE_RIGHT, -0.08);
  }

  // if (edge_flag_) {
  //   // TODO: 改成配置文件
  //   if (refer_path_->wpis[refer_index_].edge_dist > 0) {
  //     stop_shape_.setInc(SHAPE_LEFT, -0.15);
  //   } else {
  //     stop_shape_.setInc(SHAPE_RIGHT, -0.15);
  //   }
  // }

  double x_max = INT_MIN;
  double x_min = INT_MAX;
  double y_max = INT_MIN;
  double y_min = INT_MAX;
  stop_shape_.getBoundingbox(x_max, x_min, y_max, y_min);

  auto cur_pose = RobotInfo::getPtrInstance()->getCurrentPose();

  Eigen::Matrix<double, 2, 3> T_wb;
  T_wb << cos(cur_pose.getYaw()), -sin(cur_pose.getYaw()), cur_pose.getX(),
      sin(cur_pose.getYaw()), cos(cur_pose.getYaw()), cur_pose.getY();

  for (double x = x_min; x < x_max; x += ptr_config_->costmap_traverse_inc) {
    for (double y = y_min; y < y_max; y += ptr_config_->costmap_traverse_inc) {
      Eigen::Vector2d point_in_world =
          T_wb.block<2, 2>(0, 0) * Eigen::Vector2d(x, y) +
          T_wb.block<2, 1>(0, 2);
      if (checkPointCostValue(point_in_world) == 254) {
        if (stop_shape_.checkPonitInside(x, y)) {
          LOG(WARNING) << "costmap stop point: " << x << ", " << y;
          return true;
        }
      }
    }
  }
  stop_shape_.clearInc();
  return false;
}

int SpeedDecisionBase::checkPointCostValue(const Eigen::Vector2d &point) {
  if (ptr_costmap_2d_ == nullptr) {
    LOG(ERROR) << "not set costmap";
    return 255;
  }
  std::lock_guard<std::mutex> lock(costmap_mutex_);

  return ptr_costmap_2d_->getCostWithMapPose(point(0), point(1));
}

Velocity SpeedDecisionBase::smoothVel(const Velocity &vel) {
  Velocity smooth_vel = vel;
  if (fabs(vel.d_x) > 0.001) {
    if (ptr_config_->max_v_inc > 0 && ptr_config_->max_v_dec < 0) {
      if (vel.d_x - last_vel_.d_x > ptr_config_->max_v_inc) {
        smooth_vel.d_x = last_vel_.d_x + ptr_config_->max_v_inc;
      } else if (vel.d_x - last_vel_.d_x < ptr_config_->max_v_dec) {
        smooth_vel.d_x = last_vel_.d_x + ptr_config_->max_v_dec;
      }
      last_vel_ = smooth_vel;
    } else {
      LOG(INFO) << "wrong max_v_inc or max_v_dec";
      return smooth_vel;
    }
  }
  return smooth_vel;
}

void SpeedDecisionBase::checkMinVel(Velocity &vel) {
  bool check_v = false;
  if (fabs(vel.d_x) > 0.1 * ptr_config_->min_v &&
      vel.d_x < ptr_config_->min_v && vel.d_x > 0) {
    vel.d_x = ptr_config_->min_v;
    check_v = true;
  }
  if (fabs(vel.d_x) > 0.1 * ptr_config_->min_v &&
      vel.d_x > -ptr_config_->min_v && vel.d_x < 0) {
    vel.d_x = -ptr_config_->min_v;
    check_v = true;
  }
  if (check_v) {
    if (vel.d_yaw < ptr_config_->min_w && vel.d_yaw > 0) {
      vel.d_yaw = ptr_config_->min_w;
    }
    if (vel.d_yaw > -ptr_config_->min_w && vel.d_yaw < 0) {
      vel.d_yaw = -ptr_config_->min_w;
    }
  }
}

Eigen::Vector3d SpeedDecisionBase::rotation2rpy(const Eigen::Matrix3d &R) {
  double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
  bool singular = sy < 1e-6;  // true: `Y`方向旋转为`+/-90`度
  double x, y, z;
  if (!singular) {
    x = atan2(R(2, 1), R(2, 2));
    y = atan2(-R(2, 0), sy);
    z = atan2(R(1, 0), R(0, 0));
  } else {
    x = atan2(-R(1, 2), R(1, 1));
    y = atan2(-R(2, 0), sy);
    z = 0;
  }
  return Eigen::Vector3d(x, y, z);
}

bool SpeedDecisionBase::getEdgeFlag() {
  static bool last_flag = false;
  if (refer_path_ == nullptr) {
    LOG(ERROR) << "refer_path_ is nullptr";
    edge_flag_ = false;
    return false;
  }
  if (refer_path_->wps.size() > refer_index_) {
    if (refer_path_->wpis[refer_index_].is_edge_wise != 0 &&
        RobotInfo::getPtrInstance()->getMissionType() == MISSIONTYPE::EDGE) {
      edge_flag_ = true;
    } else {
      edge_flag_ = false;
    }
  } else {
    LOG(WARNING) << "refer_path_is empty";
  }
  if (last_flag != edge_flag_) {
    last_flag = edge_flag_;
    LOG(WARNING) << "change edge mode " << edge_flag_ << " wise is "
                 << refer_path_->wpis[refer_index_].is_edge_wise;
  }
  return edge_flag_;
}

double SpeedDecisionBase::getRecoverFactor(
    const RecoveDir dir_key, Eigen::Matrix<double, 2, 3> *T_wb_ptr) {
  // 获取并校验图层
  if (m_recover_coverage_.find(dir_key) == m_recover_coverage_.end()) {
    LOG(ERROR) << "can`t find recover coverage";
    return 255.0;
  }
  std::string dir = m_recover_coverage_[dir_key];

  // 清空上次的缓存
  recover_shape_.clearInc();
  danger_shape_.clearInc();

  // 激光
  {
    PncMap pnc_map;
    Pose2d robot_pose = RobotInfo::getPtrInstance()->getCurrentPose();
    // 调整激光脱困参数
    // if (RobotInfo::getPtrInstance()->getMissionType() ==
    // MISSIONTYPE::ELVATOR) {
    if (pnc_map.InElevatorArea(
            math_utils::Vec2d(robot_pose.getX(), robot_pose.getY())) ||
        RobotInfo::getPtrInstance()->getMissionType() == MISSIONTYPE::ELVATOR) {
      // 电梯任务调整致命框
      LOG(INFO) << "elevator shrink shape";
      danger_shape_.setInc(SpeedDir::SHAPE_LEFT, -0.03);
      danger_shape_.setInc(SpeedDir::SHAPE_RIGHT, -0.03);
      if (dir == "right" || dir == "left") {
        return 254.0;
      }
    }
    if (RobotInfo::getPtrInstance()->getMissionType() == MISSIONTYPE::EDGE) {
      // 贴边任务调整停障框
      LOG(INFO) << "edge shrink shape";
      danger_shape_.setInc(SpeedDir::SHAPE_LEFT, -0.03);
      danger_shape_.setInc(SpeedDir::SHAPE_RIGHT, -0.03);
    }

    // search danger scan
    std::lock_guard<std::mutex> lock_scan(scan_mutex_);
    std::vector<Eigen::Vector2d> crash_point;
    if (scan_.empty()) {
      LOG(ERROR) << "scan size error";
    }
    if (danger_shape_.checkPonitInside(scan_, dir, &crash_point)) {
      LOG(WARNING) << dir << " scan can`t go " << dir << " x "
                   << crash_point[0](0) << " y " << crash_point[0](1);
      return 254.0;
    }
  }

  // psd
  if (dir == "left") {
    std::vector<float> psdValue = RobotInfo::getPtrInstance()->getPSDValue();
    if (!psdValue.empty()) {
      float backPSD = psdValue.at(3);
      if (backPSD > -0.032) {
        // psd不可以旋转
        LOG(WARNING) << dir << " stop for psd forward " << psdValue.at(2)
                     << " back " << backPSD;
        return 254.0;
      }
    }
  }
  // costmap
  {
    // get T
    Eigen::Matrix<double, 2, 3> T_wb;
    if (T_wb_ptr == nullptr) {
      auto cur_pose = RobotInfo::getPtrInstance()->getCurrentPose();
      T_wb << cos(cur_pose.getYaw()), -sin(cur_pose.getYaw()), cur_pose.getX(),
          sin(cur_pose.getYaw()), cos(cur_pose.getYaw()), cur_pose.getY();
      T_wb_ptr = &T_wb;
    }

    // get boundingbox
    double x_max = INT_MIN;
    double x_min = INT_MAX;
    double y_max = INT_MIN;
    double y_min = INT_MAX;
    danger_shape_.setInc(SHAPE_LEFT, -0.05);
    danger_shape_.setInc(SHAPE_RIGHT, -0.05);
    danger_shape_.getBoundingbox(x_max, x_min, y_max, y_min);

    // search danger point costmap
    auto v_danger_point = danger_shape_.getStopShape(dir);
    for (int i = 0; i < v_danger_point.size(); i++) {
      if (v_danger_point[i](0) < x_max && v_danger_point[i](0) > x_min &&
          v_danger_point[i](1) < y_max && v_danger_point[i](1) > y_min) {
        Eigen::Vector2d point_in_world =
            T_wb_ptr->block<2, 2>(0, 0) * v_danger_point[i] +
            T_wb_ptr->block<2, 1>(0, 2);

        if (checkPointCostValue(point_in_world) == 254) {
          LOG(WARNING) << dir << " costmap stop point: " << v_danger_point[i](0)
                       << " " << v_danger_point[i](1);
          return 254.0;
        }
      }
    }

    // get point from stopShape
    std::vector<Eigen::Vector2d> *v_recover_point_ptr = nullptr;
    recover_shape_.getStopShapeOriginal(&v_recover_point_ptr, dir);
    size_t v_recover_point_size = v_recover_point_ptr->size();

    // 计算因子
    double cost_sum = 0;
    for (int i = 0; i < v_recover_point_size; i++) {
      Eigen::Vector2d point_in_world =
          T_wb_ptr->block<2, 2>(0, 0) * (*v_recover_point_ptr)[i] +
          T_wb_ptr->block<2, 1>(0, 2);
      int cost = checkPointCostValue(point_in_world);
      if (cost == 254) {
        LOG(INFO) << "find obs in recover x:" << (*v_recover_point_ptr)[i](0)
                  << " y " << (*v_recover_point_ptr)[i](1);
        return 250.0;
      } else {
        cost_sum += cost;
      }
    }

    if (v_recover_point_size == 0) {
      LOG(ERROR) << "v_recover_point size error";
    }

    return cost_sum / v_recover_point_size;
  }
}

double SpeedDecisionBase::getArcRecoverFactor(
    const RecoveDir dir_key, Eigen::Matrix<double, 2, 3> *T_wb_ptr) {
  if (m_recover_coverage_.find(dir_key) == m_recover_coverage_.end()) {
    LOG(ERROR) << "can`t find recover coverage";
    return 255.0;
  }
  std::string dir = m_recover_coverage_[dir_key];
  arc_recover_shape_.clearInc();
  // scan
  // 激光
  {
    std::vector<Eigen::Vector2d> crash_point;
    if (scan_.empty()) {
      LOG(ERROR) << "scan size error";
    }
    if (arc_recover_shape_.checkPonitInside(scan_, dir, &crash_point)) {
      LOG(WARNING) << dir << " recover scan can`t go " << dir << " x "
                   << crash_point[0](0) << " y " << crash_point[0](1);
      return 254.0;
    }
  }

  // costmap
  {
    if (dir == "right") {
      arc_recover_shape_.setInc(SpeedDir::SHAPE_LEFT, -0.12);
    } else if (dir == "left") {
      arc_recover_shape_.setInc(SpeedDir::SHAPE_RIGHT, -0.12);
    }

    // 初始化旋转矩阵
    Eigen::Matrix<double, 2, 3> T_wb;
    if (T_wb_ptr == nullptr) {
      auto cur_pose = RobotInfo::getPtrInstance()->getCurrentPose();
      T_wb << cos(cur_pose.getYaw()), -sin(cur_pose.getYaw()), cur_pose.getX(),
          sin(cur_pose.getYaw()), cos(cur_pose.getYaw()), cur_pose.getY();
      T_wb_ptr = &T_wb;
    }

    // 计算因子
    // get boundingbox
    double x_max = INT_MIN;
    double x_min = INT_MAX;
    double y_max = INT_MIN;
    double y_min = INT_MAX;
    double cost_sum = 0;
    auto v_recover_point = arc_recover_shape_.getStopShape(dir);
    size_t v_recover_point_size = v_recover_point.size();

    arc_recover_shape_.getBoundingbox(x_max, x_min, y_max, y_min);

    for (int i = 0; i < v_recover_point_size; i++) {
      if (v_recover_point[i](0) < x_max && v_recover_point[i](0) > x_min &&
          v_recover_point[i](1) < y_max && v_recover_point[i](1) > y_min) {
        Eigen::Vector2d point_in_world =
            T_wb_ptr->block<2, 2>(0, 0) * v_recover_point[i] +
            T_wb_ptr->block<2, 1>(0, 2);
        int cost = checkPointCostValue(point_in_world);
        if (cost == 254) {
          LOG(WARNING) << dir
                       << " arc costmap stop point: " << v_recover_point[i](0)
                       << " " << v_recover_point[i](1);
          return 254.0;
        } else {
          cost_sum += cost;
        }
      }
    }

    if (v_recover_point_size == 0) {
      LOG(ERROR) << "v_recover_point size error";
    }

    return cost_sum / v_recover_point_size;
  }
  return 254.0;
}

std::vector<Eigen::Vector2d> SpeedDecisionBase::getBoundingbox(
    const std::string shape_name) {
  double x_max = INT_MIN;
  double x_min = INT_MAX;
  double y_max = INT_MIN;
  double y_min = INT_MAX;

  if (shape_name == "recover") {
    recover_shape_.getBoundingbox(x_max, x_min, y_max, y_min);
  } else if (shape_name == "danger") {
    danger_shape_.getBoundingbox(x_max, x_min, y_max, y_min);
  } else if (shape_name == "slow") {
    slow_shape_.getBoundingbox(x_max, x_min, y_max, y_min);
  } else {
    stop_shape_.getBoundingbox(x_max, x_min, y_max, y_min);
  }

  LOG(INFO) << "shape_name " << shape_name << " left_top: " << x_max << ", "
            << y_max << " right_under: " << x_min << ", " << y_min;
  return {{x_max, y_max}, {x_min, y_min}};
}

bool SpeedDecisionBase::checkFrontDanger() {
  return (getRecoverFactor(RecoveDir::RECOVER_FRONT) > 252);
}

bool SpeedDecisionBase::checkOutofRecovery() {
  Eigen::Vector2d point(RobotInfo::getPtrInstance()->getCurrentPose().getX(),
                        RobotInfo::getPtrInstance()->getCurrentPose().getY());
  if (edge_flag_) {
    if (checkPointCostValue(point) < 130 && isObstacleInStopBound() == false) {
      return true;
    }
  } else {
    if (checkPointCostValue(point) < 130 && isObstacleInStopBound() == false &&
        getRecoverFactor(RecoveDir::RECOVER_FRONT) < 250) {
      return true;
    }
  }
  return false;
}

}  // namespace CVTE_BABOT
