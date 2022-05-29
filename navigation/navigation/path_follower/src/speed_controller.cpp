/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file speed_controller.hpp
 *
 *@brief 有关导航速度等级变换的实现接口
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2020-06-09
 ************************************************************************/

#include "speed_controller.hpp"
#include <glog/logging.h>
#include "costmap_mediator.hpp"

namespace CVTE_BABOT {

std::shared_ptr<SpeedController> SpeedController::speed_ctrl_ptr_ = nullptr;
SpeedController::SpeedController() {}

std::shared_ptr<SpeedController> SpeedController::getPtrInstance() {
  if (speed_ctrl_ptr_.get() == nullptr) {
    speed_ctrl_ptr_.reset(new SpeedController());
  }
  return speed_ctrl_ptr_;
}

SpeedLevel SpeedController::getLocalizationSpeedLevel() {
  std::lock_guard<std::mutex> lock(localization_speed_level_mutex_);
  return localization_speed_level_;
}

SpeedLevel SpeedController::getCostmapSpeedLevel() {
  std::lock_guard<std::mutex> lock(costmap_speed_level_mutex_);
  return CostmapMediator::getPtrInstance()->getSpeedLevel();
}

SpeedLevel SpeedController::getMissionManagerSpeedLevel() {
  std::lock_guard<std::mutex> lock(mission_manager_level_mutex_);
  return mission_manager_level_;
}

SpeedLevel SpeedController::getMarkSpeedLevel() {
  std::lock_guard<std::mutex> lock(mark_speed_level_mutex_);
  return mark_speed_level_;
}

void SpeedController::calcLocalizaitonSpeedLevel(const double &cov) {
  std::lock_guard<std::mutex> lock(localization_speed_level_mutex_);
  // 3.0 和 10.0
  // 这两个值是张伟固态激光定位拍板的参数，不一定适用于所有定位协方差
  // -0.01是为了防止接近0.0的非常小的指数值，仿真amcl会出现
  if (-0.01 <= cov && cov < 3.0) {
    localization_speed_level_ = SpeedMax;
  } else if (3.0 < cov && cov < 10.0) {
    localization_speed_level_ = SpeedTwo;
  } else {
    localization_speed_level_ = Stop;
  }
}

void SpeedController::calcMarkSpeedLevel(SpeedLevel level) {
  std::lock_guard<std::mutex> lock(mark_speed_level_mutex_);
  mark_speed_level_ = level;
}

void SpeedController::calcMissionMangerSpeedLevel(const unsigned int &level) {
  std::lock_guard<std::mutex> lock(mission_manager_level_mutex_);
  mission_manager_level_ = intToSpeedLevel(level);
}

SpeedLevel SpeedController::intToSpeedLevel(const unsigned int &level) {
  SpeedLevel temp_level = SpeedMax;
  switch (level) {
    case 0:
      temp_level = Stop;
      break;
    case 1:
      temp_level = SpeedOne;
      break;
    case 2:
      temp_level = SpeedTwo;
      break;
    case 3:
      temp_level = SpeedThree;
      break;
    case 4:
      temp_level = SpeedMax;
      break;
    default:
      LOG(ERROR) << "Mission manager set a unsupport speedlevel: " << level;
      break;
  }
  return temp_level;
}

bool SpeedController::adjustSpeedLevel(SpeedCtrlRes &speed_res) {
  SpeedLevel costmap_level = getCostmapSpeedLevel();
  // SpeedLevel costmap_level = SpeedMax;
  SpeedLevel localization_level = getLocalizationSpeedLevel();
  SpeedLevel mission_level = getMissionManagerSpeedLevel();
  SpeedLevel mark_level = getMarkSpeedLevel();

  LOG(INFO) << "Speed Level Current: " << current_level_
            << "(co: " << costmap_level << ", lo: " << localization_level
            << ", mi: " << mission_level << ", ma: " << mark_level << ")";

  if (costmap_level == Stop) {
    current_level_ = costmap_level;
    speed_res.stop_by_who = "costmap";
    return true;
  }

  if (localization_level == Stop) {
    current_level_ = localization_level;
    speed_res.stop_by_who = "localization";
    return true;
  }

  if (mission_level == Stop) {
    current_level_ = mission_level;
    speed_res.stop_by_who = "mission";
    return true;
  }

  if (mark_level == Stop) {
    current_level_ = mark_level;
    speed_res.stop_by_who = "marker";
    return true;
  }

  // 权衡几个设置导航速度等级的来源，按照速度等级最低的为准
  SpeedLevel sensor_level =
      (costmap_level < localization_level) ? costmap_level : localization_level;
  SpeedLevel logic_level =
      (mark_level < mission_level) ? mark_level : mission_level;
  SpeedLevel final_level =
      (sensor_level < logic_level) ? sensor_level : logic_level;

  if (current_level_ == Stop && final_level != Stop) {
    // 如果当前不是停障状态,且已经恢复则开始从零加速
    current_level_ = None;
  } else if ((current_level_ < final_level) &&
             (++adjust_level_count_ % 1 != 0)) {
    // 默认此函数的运行频率是5Hz， 除了停止与减速马上响应之外
    // 其它提速的响应延迟响应，保证有一个平缓加速过程
    speed_res.stop_by_who = "";
    return false;
  }
  adjust_level_count_ = 0;

  // 如果当前速度比期望速度要大，即需要降速
  if (current_level_ == final_level) {
    speed_res.stop_by_who = "";
    return false;
  } else if (current_level_ > final_level) {
    current_level_ = final_level;
  } else if (current_level_ < final_level) {
    current_level_ = current_level_ + 1;
  }
  LOG(INFO) << "current_level_ level: " << static_cast<int>(current_level_);

  switch (current_level_) {
    case None:
    case Stop: {
      speed_res.stop_by_who = "costmap";
      LOG(INFO) << "Still stopping.";
      return true;
    } break;
    case SpeedOne:
      setSpeedResultPercent(speed_res, 0.25, 1.0);
      break;
    case SpeedTwo:
      setSpeedResultPercent(speed_res, 0.5, 1.0);
      break;
    case SpeedThree:
      setSpeedResultPercent(speed_res, 0.75, 1.0);
      break;
    case SpeedMax:
      setSpeedResultPercent(speed_res, 1.0, 1.0);
      break;
    default:
      LOG(ERROR) << "adjust SpeedLevel something wrong: " << current_level_;
  }

  return false;
}

void SpeedController::setSpeedResultPercent(SpeedCtrlRes &speed_res,
                                            const double &vel_percent,
                                            const double &th_percent) {
  speed_res.x_vel = max_velocity_.d_x * vel_percent;
  speed_res.y_vel = max_velocity_.d_y * vel_percent;
  speed_res.th_vel = max_velocity_.d_yaw * th_percent;
  speed_res.stop_by_who = "nobody";
  LOG(INFO) << "Set ctrl speed: " << speed_res.x_vel << ", "
            << speed_res.th_vel;
}

}  // namespace CVTE_BABOT