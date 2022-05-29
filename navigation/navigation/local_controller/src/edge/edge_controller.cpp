/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file edge_controller.cpp
 *
 *@brief 贴边控制算法的头文件   控制器 --> lypu
 *
 *@modified by
 *
 *@author chenweijian(chenweijian@cvte.com)
 *@version Navigation-v2.0
 *@data 2021-12-02
 ************************************************************************/
#include "edge/edge_controller.hpp"
#include "float.h"
#include "robot_info.hpp"

namespace CVTE_BABOT {

EdgeController::EdgeController() {
  ptr_navigation_mediator_ = NavigationMediator::getPtrInstance();
  lypu_ptr_ = std::make_shared<LYPULocalController>();
  getParams();
  last_close_time_ = time(NULL);
  very_close_flag_ = false;
}

MoveCommand::MoveCommandStatus EdgeController::computeMoveComand(
    MoveCommand &cmd) {
  if (local_path_->wps.empty()) {
    LOG(INFO) << "MoveCommand::MoveCommandStatus::ERROR1" << std::endl;
    return MoveCommand::MoveCommandStatus::ERROR;
  }

  Pose2d current_pose = RobotInfo::getPtrInstance()->getCurrentPose();
  TrackCtrl result;
  size_t find_index = 0;
  std::lock_guard<std::mutex> lock(path_mutex_);

  if (very_close_flag_ == true) {
    if (time(NULL) - last_close_time_ >= 2) {
      // 上一次离墙较近时，延时清空角度与角速度  单位:s
      LOG(WARNING) << "out of wall ";
      very_close_flag_ = false;
    }
  }

  if (findTrackTarget(current_pose, find_index, last_target_index_)) {
    // 获取下一个轨迹点
    bool change_target = false;
    Pose2d next_target = local_path_->wps[find_index];
    Pose2d inv_current_pose = current_pose.inverse();
    Pose2d local_next_target = inv_current_pose * next_target;
    double edge_dist = fabs(local_path_->wpis[find_index].edge_dist);
    double target_v = local_path_->wpis[find_index].v;
    double target_w = local_path_->wpis[find_index].w;

    /*
    C3上的安装参数：
          joint_x="0.1125"
          joint_y="-0.223"
          joint_z="-0.05"
          joint_roll="0"
          joint_pitch="0"
          joint_yaw="${-5*PI/12}"
    */
    // TODO: 修改到传感器配置
    float psd_install_y = -0.23;
    float psd_install_x = 0.062;
    float psd_install_yaw = -5 * M_PI / 12;
    Pose2d install_pose_forward(psd_install_x, psd_install_y, psd_install_yaw);

    std::vector<float> v_psd_value = RobotInfo::getPtrInstance()->getPSDValue();
    double psd_value = 0.7;
    if (!v_psd_value.empty()) {  // 有数据才使用
      psd_value = -v_psd_value.at(2);
    }

    if (fabs(AngleCalculate::rad2degree(local_next_target.getYaw())) < 100 &&
        local_next_target.distanceTo(Pose2d(0, 0, 0)) < 0.6 &&
        local_path_->wpis[find_index].is_edge_wise == 2 &&
        local_path_->wpis.back().path_length -
                local_path_->wpis[find_index].path_length >
            0.8) {
      // 贴边传感器数据正常
      if (fabs(psd_value) > 0.02 && fabs(psd_value) < 0.4) {
        // FIXME: 对照kb的图进行查看

        // a).psd 测量点到 baselink 距离
        float delta_yaw = local_next_target.getYaw();

        // b).point_A
        double point_A_p =
            psd_value + install_pose_forward.distanceTo(Pose2d(0, 0, 0));

        // c).计算 baselink 到 期望路径距离 (psd射线上) (point_B)
        double point_B_p =
            point_A_p -
            (edge_dist /
             cos(delta_yaw + (M_PI / 2 - fabs(psd_install_yaw))));  // LWT

        // d).极坐标计算 (point_B)
        double point_B_x = point_B_p * cos(psd_install_yaw);
        double point_B_y = point_B_p * sin(psd_install_yaw);

        // e).根据极坐标计算期望直线
        double a = tan(delta_yaw);
        double b = -1;
        double c = -(a * point_B_x) - (b * point_B_y);

        // f).求垂足
        double foot_point_x = 0;
        double foot_point_y = 0;
        if (fabs(c) < 1e-13) {
          // 垂足与直线重合
          foot_point_x = 0;
          foot_point_y = 0;
        } else {
          foot_point_x = (-a * c) / (a * a + b * b);
          foot_point_y = (-b * c) / (a * a + b * b);
        }

        // g.)求前瞻点
        double forward_point = 0.3;
        float refer_x = foot_point_x + forward_point * cos(delta_yaw);
        float refer_y = foot_point_y + forward_point * sin((delta_yaw));
        if (fabs(refer_y - local_next_target.getY()) < 0.6) {
          local_next_target.setY(refer_y);
          local_next_target.setX(refer_x);
          target_w = 0;
          local_next_target.setYaw(0);
          change_target = true;
          if (refer_y > -0.05 && delta_yaw < 0.1) {
            LOG(INFO) << "** very close **";

            very_close_flag_ = true;
            last_close_time_ = time(NULL);
          }
        } else {
          LOG(WARNING) << "local traget too fat refer_y " << refer_y
                       << " local " << local_next_target.getY();
        }
      } else {
        LOG(WARNING) << "psd data : " << psd_value;
      }
    } else {
      if (local_path_->wpis[find_index].is_edge_wise == 2) {
        LOG(WARNING) << "too fat to refer path length : "
                     << local_next_target.distanceTo(Pose2d(0, 0, 0))
                     << " yaw: "
                     << AngleCalculate::rad2degree(local_next_target.getYaw());
      } else {
        LOG(INFO) << "paths edge "
                  << local_path_->wpis[find_index].is_edge_wise;
      }
    }

    // 将前瞻点重新换算为全局坐标
    next_target = current_pose * local_next_target;

    // 位置环闭环控制
    if (change_target == false) {
      if (last_ctrl_mode_ == STANLEY_) {
        // 切换模式 清空缓存
        LOG(INFO) << "lypu clear";
        lypu_ptr_->clearCache();
      }

      if (very_close_flag_) {
        local_next_target.setYaw(0);
        target_w = 0;
        next_target = current_pose * local_next_target;
        target_v = target_v > 0.2 ? 0.2 : target_v;
      }
      // lypu控制器
      lypu_ptr_->trackingLYPU(next_target, current_pose, target_v, target_w,
                              result);

      last_ctrl_mode_ = LYPU_;
    } else {
      // stanley闭环控制
      trackingStanley(next_target, current_pose, target_v, target_w, result);
      result.v_velocity = result.v_velocity > 0.2 ? 0.2 : result.v_velocity;
      last_ctrl_mode_ = STANLEY_;
    }

    LOG(INFO) << "next_local_target: x: " << local_next_target.getX()
              << "  y:" << local_next_target.getY()
              << "  yaw:" << local_next_target.getYaw();

    LOG(INFO) << "Calc:  V = " << result.v_velocity
              << "  W = " << result.w_velocity;

    LOG(INFO) << "Ctrl mode " << last_ctrl_mode_;
    // 速度输出
    cmd.setVelX(result.v_velocity);
    cmd.setVelY(0);
    cmd.setVelTH(result.w_velocity);
    cmd.setCarrotPose(next_target);

    return MoveCommand::MoveCommandStatus::OKAY;
  } else {
    return MoveCommand::MoveCommandStatus::ERROR;
  }
  LOG(INFO) << "******  once edge  ******";
}

double EdgeController::getRemainCtrlPathLength() {
  std::lock_guard<std::mutex> lock(path_mutex_);
  if (local_path_ == nullptr) {
    return 0.0;
  }
  double dist = local_path_->wpis.back().path_length -
                local_path_->wpis[last_target_index_].path_length;
  LOG(INFO) << "get remain ctrl path length: " << dist;
  return dist;
}

size_t EdgeController::getReferIndex() {
  return last_target_index_;
}

void EdgeController::trackingStanley(const Pose2d &traget_pos,
                                     const Pose2d &current_pos,
                                     const double &target_v,
                                     const double &target_w,
                                     TrackCtrl &result) {
  double delta_ss = kag_ * target_v * target_w;
  Velocity cur_vel = getCurrentVelocity();
  double dX = traget_pos.getX() - current_pos.getX();
  double dY = traget_pos.getY() - current_pos.getY();

  // \psi
  double dQ =
      AngleCalculate::angle_diff(traget_pos.getYaw(), current_pos.getYaw());

  double dYe =
      (-1.0) * dX * sin(current_pos.getYaw()) + dY * cos(current_pos.getYaw());

  //                    经验值                                    v(t)
  double steer_angle = 0.3 * dQ + atan(kp_e_ * dYe / (kp_soft_ + target_v)) +
                       // 系数 × 路径上的先验角速度（搜路径的时候搜出来的）
                       kd_yaw_ * (target_w);

  steer_angle = checkLimitSteerAngle(steer_angle);

  // 计算线速度
  double vel = target_v * cos(dQ) + kp_v_ * fabs(dX);

  LOG(INFO) << "steer_angle : " << steer_angle << " vel " << vel;
  // 换算控制中心
  result = changeCtrlOutput(vel, steer_angle);
  last_steer_angle_ = steer_angle;
}

void EdgeController::getParams() {
  if (!ptr_navigation_mediator_->isParameterReady()) {
    LOG(ERROR) << "Is mediator's Parameter initialized? ";
    return;
  }

  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.edge.kag", kag_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.edge.kp_e", kp_e_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.edge.kp_soft", kp_soft_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.edge.kd_yaw", kd_yaw_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.edge.kp_v", kp_v_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.edge.car_length", car_length_,
      1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.edge.max_w", max_w_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.edge.max_vel", max_vel_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.edge.min_w", min_w_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.edge.min_vel", min_vel_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.edge.sensor", edge_sensor_,
      std::string("LIDAR"));

  LOG(INFO) << "read edge params: ";
  LOG(INFO) << "kag_: " << kag_ << " kp_e_: " << kp_e_
            << " kp_soft_: " << kp_soft_ << " kd_yaw_: " << kd_yaw_
            << " kp_v_: " << kp_v_;
  LOG(INFO) << "car_length: " << car_length_;
  LOG(INFO) << "max_vel: " << max_vel_ << " max_w:" << max_w_
            << " min_vel: " << min_vel_ << " min_w: " << min_w_;
  LOG(INFO) << "edge_sensor: " << edge_sensor_;

  setMoveSpeedRange(0.0, max_vel_, 0.0, 0.0, 0.0, max_w_);
  u_steer_angle_ = 0.48 * M_PI;  // 舵轮最大角度(87)
  l_steer_angle_ = -0.48 * M_PI;
  last_steer_angle_ = 0.0;
}

double EdgeController::checkLimitSteerAngle(double steer_angle) const {
  if (steer_angle > 0) {
    return steer_angle < u_steer_angle_ ? steer_angle : u_steer_angle_;
  } else {
    return steer_angle > l_steer_angle_ ? steer_angle : l_steer_angle_;
  }
}

bool EdgeController::setPath(std::shared_ptr<SubPath> path,
                             size_t begin_index) {
  if (path == nullptr || path->wps.empty()) {
    LOG(ERROR) << "No sub Paths.";
    return false;
  }

  // 直接获取路径集合中的当前路径，外面根据情况做逻辑判断挑选需要执行的路径
  std::lock_guard<std::mutex> lock(path_mutex_);
  local_path_ = path;
  if (local_path_->wps.size() <= 0) {
    LOG(ERROR) << "Lypu Received an Empty Target Plan.";
    return false;
  }
  if (local_path_->wps.size() != local_path_->wpis.size()) {
    LOG(ERROR) << "Lypu Received an Plan whitout path info.";
    return false;
  }
  LOG(INFO) << "set subPath size = " << local_path_->wps.size() << ", "
            << local_path_->wpis.size();

  last_target_index_ = begin_index;
  return true;
}

bool EdgeController::findTrackTarget(const Pose2d &c_pose,
                                     size_t &path_point_index,
                                     size_t &last_point_index,
                                     double front_distance, double sum_dist) {
  if (local_path_->wps.empty()) {
    LOG(ERROR) << "TrackPath is empty.";
    return false;
  }

  if (local_path_->wps.size() == 1) {
    path_point_index = 0;
    LOG(INFO) << "path size 1 = " << local_path_->wps.size();
    return false;
  }
  // 查找路径上离机器人最近的点
  size_t min_index = last_point_index;
  double distance = 0.0;
  double min_distance = DBL_MAX;

  // 修改为只在 1m 范围 找目标点

  for (size_t i = last_point_index;
       i < local_path_->wps.size() - 1 &&
       local_path_->wpis[i].path_length -
               local_path_->wpis[last_point_index].path_length <
           sum_dist;
       i++) {
    distance = c_pose.distanceTo(local_path_->wps[i]);
    if (min_distance > distance) {
      min_distance = distance;
      min_index = i;
    }
  }

  last_point_index = min_index;
  // LOG_EVERY_N(INFO, LOG_HZ)
  LOG(INFO) << "Minest_distance = " << min_distance
            << "  min_index = " << min_index;

  // TODO: 胡萝卜点在当前位置后面时将导致机器人不移动

  // 离机器最近的
  for (size_t i = min_index; i < local_path_->wps.size() - 1; i++) {
    if (local_path_->wpis[i].path_length -
            local_path_->wpis[min_index].path_length >
        front_distance) {
      path_point_index = i;
      LOG(INFO) << "find targer point: " << path_point_index;
      return true;
    }
  }
  path_point_index = local_path_->wps.size() - 1;
  LOG(INFO) << " find the last point as target.";
  return true;
}

TrackCtrl EdgeController::changeCtrlOutput(double vel, double steer_angle_) {
  std::lock_guard<std::mutex> lock(steer_angle_mutex_);
  TrackCtrl ctrl;
  double l_r = 0.3;
  double belta = atan(l_r * tan(steer_angle_) / car_length_);
  ctrl.v_velocity = vel;
  ctrl.w_velocity = 0.5 * cos(belta) * tan(steer_angle_) / car_length_;
  checkMaxVel(ctrl.v_velocity, ctrl.w_velocity);
  checkMinVel(ctrl.v_velocity, ctrl.w_velocity);
  return ctrl;
}

double EdgeController::changeToSteerAngle(double vel, double w) {
  return atan(w * car_length_ / vel);
}

void EdgeController::checkMaxVel(double &v_vel, double &w_vel) const {
  if (v_vel > max_vel_) {
    LOG(INFO) << "faster than max_v : " << max_vel_ << " v " << v_vel;
    v_vel = max_vel_;
  } else if (v_vel < -0.2 * max_vel_) {
    LOG(INFO) << "back faster than max_v : " << -0.2 * max_vel_ << " v "
              << v_vel;
    v_vel = -0.2 * max_vel_;
  }
  if (std::fabs(w_vel) > max_w_) {
    LOG(INFO) << "back faster than w_vel : " << w_vel << " w " << w_vel;
    w_vel = w_vel > 0 ? max_w_ : -max_w_;
  }
}

void EdgeController::checkMinVel(double &v_vel, double &w_vel) const {
  bool adjust_min_v =
      false;  // 调整线速度标志位，只有线速度被最小值调整时再考虑w是否需要被修改
  if (std::fabs(v_vel) < min_vel_) {
    LOG(INFO) << "slower min_v: " << min_vel_ << " v " << v_vel;
    v_vel = v_vel > 0 ? min_vel_ : -min_vel_;
    adjust_min_v = true;
  }
  if (adjust_min_v && std::fabs(w_vel) < min_w_ &&
      std::fabs(w_vel) > 0.2 * min_w_) {
    LOG(INFO) << "slower min_w: " << min_w_ << " w " << w_vel;
    w_vel = w_vel > 0 ? min_w_ : -min_w_;
  }
}
}  // namespace CVTE_BABOT
