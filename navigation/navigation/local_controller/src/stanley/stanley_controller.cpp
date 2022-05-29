#include "stanley/stanley_controller.hpp"
#include "float.h"
#include "robot_info.hpp"

namespace CVTE_BABOT {

StanleyController::StanleyController() {
  ptr_navigation_mediator_ = NavigationMediator::getPtrInstance();
  lypu_ptr_ = std::make_shared<LYPULocalController>();
  getParams();
  last_close_time_ = time(NULL);
  very_close_flag_ = false;
}

MoveCommand::MoveCommandStatus StanleyController::computeMoveComand(
    MoveCommand &cmd) {
  if (local_path_->wps.empty()) {
    LOG(INFO) << "MoveCommand::MoveCommandStatus::ERROR1" << std::endl;
    return MoveCommand::MoveCommandStatus::ERROR;
  }

  Pose2d current_pose = RobotInfo::getPtrInstance()->getCurrentPoseWithOffset();
  TrackCtrl result;
  size_t find_index = 0;
  bool edge_ctrl_flag = false;
  std::lock_guard<std::mutex> lock(path_mutex_);

  if (findTrackTarget(current_pose, find_index, last_target_index_)) {
    // 获取下一个轨迹点
    Pose2d next_target = local_path_->wps[find_index];
    double target_v = local_path_->wpis[find_index].v;
    double target_w = local_path_->wpis[find_index].w;
    // 下一个目标点 (局部坐标系)
    Pose2d inv_current_pose = current_pose.inverse();
    Pose2d local_next_target = inv_current_pose * next_target;

    if (very_close_flag_ == true) {
      if (time(NULL) - last_close_time_ >= 2) {
        // 上一次离墙较近时，延时清空角度与角速度  单位:s
        LOG(WARNING) << "out of wall ";
        very_close_flag_ = false;
      }
    }
    // psd安装位置 TODO: 将psd_install_y 修改为配置文件的值
    float psd_install_y = -0.35;
    // 根据安装位置换算psd数据
    Pose2d offset_pose =
        inv_current_pose * RobotInfo::getPtrInstance()->getCurrentPose();
    Pose2d install_pose_forward =
        offset_pose * Pose2d(psd_forward_install_, psd_install_y, 0);
    Pose2d install_pose_back =
        offset_pose * Pose2d(psd_back_install_, psd_install_y, 0);

    float forwardPSD = install_pose_forward.getY();
    float backPSD = install_pose_back.getY();

    // 如果发现时贴边路径 && 当前角速度较小 && 离终点较远
    size_t before_index = find_index - 30 < 0 ? 0 : find_index - 30;
    if (local_path_->wpis[find_index].is_edge_wise == 2 &&
        local_path_->wpis[before_index].is_edge_wise == 2 &&
        local_path_->wpis.back().path_length -
                local_path_->wpis[find_index].path_length >
            0.8) {
      double edge_dist = local_path_->wpis[find_index].edge_dist;

      if (edge_sensor_ != "PSD") {
        // 模拟数据
        std::vector<Pose2d> local_scan = RobotInfo::getPtrInstance()->getScan();

        // 转换坐标系
        Eigen::Matrix<double, 2, 3> T_bl;
        T_bl << 1, 0, 0.484, 0, 1, 0;
        for (size_t index = 0; index < local_scan.size(); index++) {
          Eigen::Vector2d scan_eigen(local_scan[index].getX(),
                                     local_scan[index].getY());
          scan_eigen =
              T_bl.block<2, 2>(0, 0) * scan_eigen + T_bl.block<2, 1>(0, 2);
          local_scan[index] = Pose2d(scan_eigen(0), scan_eigen(1), 0);
        }

        std::vector<Pose2d> forward_scan;
        std::vector<Pose2d> back_scan;

        // 获取期望点附近的雷达数据
        for (const auto &obj : local_scan) {
          if (fabs(obj.getX() - install_pose_forward.getX()) < 0.05 &&
              obj.getY() * edge_dist > 0 && fabs(obj.getY()) < 1.0) {
            forward_scan.push_back(obj);
          }
        }

        for (const auto &obj : local_scan) {
          if (fabs(obj.getX() - install_pose_back.getX()) < 0.05 &&
              obj.getY() * edge_dist > 0 && fabs(obj.getY()) < 1.0) {
            back_scan.push_back(obj);
          }
        }

        if (!forward_scan.empty()) {
          float dist_sum_y = 0;
          for (const auto &obj : forward_scan) { dist_sum_y += obj.getY(); }
          forwardPSD = (dist_sum_y / (float) forward_scan.size());
        }
        if (!back_scan.empty()) {
          float dist_sum_y = 0;
          for (const auto &obj : back_scan) { dist_sum_y += obj.getY(); }
          backPSD = (dist_sum_y / (float) back_scan.size());
        }

        LOG(INFO) << "lidar_to_psd  " << forwardPSD << "   " << backPSD;
      } else {
        // 真实数据
        std::vector<float> psdValue =
            RobotInfo::getPtrInstance()->getPSDValue();
        if (!psdValue.empty())  // 有数据才使用
        {
          // 增加横向位置
          forwardPSD = psdValue.at(2) + install_pose_forward.getY();
          backPSD = psdValue.at(3) + install_pose_back.getY();
        }
      }

      if (find_index < 10) {
        delta_yaw = local_next_target.getYaw();
      } else {
        delta_yaw = local_next_target.getYaw() * 0.1 + delta_yaw * 0.9;
      }

      // psd数据有效 && 轨迹偏差小于60度
      if (fabs(forwardPSD - install_pose_forward.getY()) > 0.02 &&
          fabs(forwardPSD - install_pose_forward.getY()) < 0.4 &&
          fabs(AngleCalculate::rad2degree(delta_yaw)) < 60) {
        // 重新计算横向误差
        float refer_y = forwardPSD - (edge_dist / cos(delta_yaw)) +
                        (install_pose_forward.getX() * tan(delta_yaw));
        float refer_x = install_pose_forward.getX();
        // 机器已经到墙边
        bool fornt_obs_flag = false;
        if (refer_y > -0.03 &&
            fabs(AngleCalculate::rad2degree(delta_yaw)) < 6) {
          // 查找前方是否存在障碍物
          std::vector<Pose2d> front_scan;
          std::vector<Pose2d> local_scan =
              RobotInfo::getPtrInstance()->getScan();
          for (const auto &obj : local_scan) {
            if (obj.getX() < 0.5 && obj.getX() > 0 &&
                fabs(obj.getY() - 0.0) < 0.32) {
              front_scan.push_back(obj);
            }
          }
          if (front_scan.size() > 4 &&
              last_target_index_ < local_path_->wps.size() - 1) {
            //发现障碍 清空 1.5米 内的贴边
            fornt_obs_flag = true;
            LOG(ERROR) << "!!!!!!  find obs in edgeMoving  !!!!!!";
            double sum = 0;
            for (int i = last_target_index_; i < local_path_->wps.size() - 1;
                 i++) {
              sum += local_path_->wps[i + 1].distanceTo(local_path_->wps[i]);
              if (sum > 1.5) {
                break;
              }
              // 清除前方点的贴边标志
              local_path_->wpis[i].is_edge_wise = 1;
              // 偏移目标点
              Pose2d path_offset = {0, 0.11, 0};
              Pose2d local_before = local_path_->wps[i];
              local_path_->wps[i] = (local_path_->wps[i] * path_offset);
            }
          }
        }
        // 障碍物路径偏移
        if (!fornt_obs_flag) {
          // 偏差较小才使用
          if (fabs(refer_y - local_next_target.getY()) < 1.0) {
            // 记录修改前的坐标
            LOG(INFO) << "before_local_target: x: " << local_next_target.getX()
                      << "  y: " << local_next_target.getY()
                      << "  yaw: " << local_next_target.getYaw();

            // 偏差限幅
            refer_y = refer_y < -0.2 ? -0.2 : refer_y;

            // 贴近墙时 后方超声波有障碍，不继续贴近墙体
            if (very_close_flag_ && backPSD > -(0.42 + 0.05) &&
                refer_y < -0.04) {
              LOG(INFO) << "back psd had obs";
              refer_y = 0;
            }

            // 根据偏差将x往前偏移 偏差越大 前移越多
            // 不清空角速度 容易在准备脱离墙边时触发停障
            local_next_target.setY(refer_y);
            local_next_target.setX(refer_x);
            local_next_target.setYaw(delta_yaw);
            target_w = 0;
            edge_ctrl_flag = true;
            // 记录偏差较小时的标志
            if (refer_y > -0.03 && delta_yaw < 0.08 &&
                backPSD > -(0.42 + 0.04)) {
              if (very_close_flag_ == false) {
                LOG(WARNING) << "** very close **";
              } else {
                LOG(INFO) << "** very close **";
              }
              very_close_flag_ = true;
              last_close_time_ = time(NULL);
            }
          } else {
            LOG(ERROR) << "refer_path is too fat referY:"
                       << local_next_target.getY() << " psdY: " << refer_y;
          }
        }
      } else {
        LOG(WARNING) << "psd not data";
      }
      LOG(INFO) << "forward psd : " << forwardPSD - install_pose_forward.getY()
                << "  back : " << backPSD - install_pose_back.getY();

      // 将前瞻点重新换算为全局坐标
      next_target = current_pose * local_next_target;
      delta_yaw = local_next_target.getYaw();
    } else {
      // 发现路径不能贴边，直接跟随路径行走
      LOG(INFO) << "use refer path";
      LOG(INFO) << "distance to end "
                << local_path_->wpis.back().path_length -
                       local_path_->wpis[find_index].path_length;
      LOG(INFO) << "is_edge_wise " << local_path_->wpis[find_index].is_edge_wise
                << " before " << local_path_->wpis[before_index].is_edge_wise;
      delta_yaw = local_next_target.getYaw();
    }

    // 位置环闭环控制
    if (local_path_->wpis[find_index].is_edge_wise != 2) {
      /*
      if (last_ctrl_mode_ == STANLEY_) {
        // 切换模式 清空缓存
        LOG(INFO) << "lypu clear";
        lypu_ptr_->clearCache();
      }
      // 重新找lypu控制点
      Pose2d current_pose_lypu = RobotInfo::getPtrInstance()->getCurrentPose();
      // 类型转换
      size_t find_index_lypu = 0;
      int64_t start = ((int64_t) last_target_index_) - 50;
      start = start < 0 ? 0 : start;
      size_t last_target_index_lypu = start;
      findTrackTarget(current_pose_lypu, find_index_lypu,
                      last_target_index_lypu, 0.1, 2);

      next_target = local_path_->wps[find_index_lypu];
      target_v = local_path_->wpis[find_index_lypu].v;
      target_w = local_path_->wpis[find_index_lypu].w;

      Pose2d local_next_target_temp = current_pose_lypu.inverse() * next_target;

      if (very_close_flag_) {
        // 离墙太近时 需要先不考虑路径角度信息
        LOG(INFO) << "** still very close **";

        double delate_yaw = local_next_target_temp.getYaw();

        LOG(INFO) << "last yaw: " << delate_yaw << " w " << target_w;
        target_w = target_w < 0 ? 0 : target_w;
        delate_yaw = delate_yaw < 0 ? 0 : delate_yaw;

        local_next_target_temp.setYaw(delate_yaw);
      } else {
        // 横向偏差较大时
        if (local_next_target_temp.getY() > 0.08) {
          if (fabs(backPSD - install_pose_forward.getY()) < 0.4) {
            LOG(INFO) << "too fat to path clear target";
            double delate_yaw = local_next_target_temp.getYaw();

            target_w = target_w < 0 ? 0 : target_w;
            delate_yaw = delate_yaw < 0 ? 0 : delate_yaw;
            local_next_target_temp.setYaw(delate_yaw);
          }
        }
      }

      next_target = current_pose_lypu * local_next_target_temp;
      // lypu控制器
      lypu_ptr_->trackingLYPU(next_target, current_pose_lypu, target_v,
                              target_w, result);
      // 可视化
      inv_current_pose = current_pose.inverse();
      local_next_target = inv_current_pose * next_target;
      // 更改控制模式
      last_ctrl_mode_ = LYPU_;
      */
      if (very_close_flag_) {
        // 离墙太近时 需要先不考虑路径角度信息
        LOG(INFO) << "** still very close **";

        double delate_yaw = local_next_target.getYaw();

        LOG(INFO) << "last yaw: " << delate_yaw << " w " << target_w;
        target_w = target_w < 0 ? 0 : target_w;
        delate_yaw = delate_yaw < 0 ? 0 : delate_yaw;

        local_next_target.setYaw(delate_yaw);
      } else {
        // 横向偏差较大时
        if (local_next_target.getY() > 0.08) {
          if (fabs(backPSD - install_pose_forward.getY()) < 0.4) {
            LOG(INFO) << "too fat to path clear target";
            double delate_yaw = local_next_target.getYaw();

            target_w = target_w < 0 ? 0 : target_w;
            delate_yaw = delate_yaw < 0 ? 0 : delate_yaw;
            local_next_target.setYaw(delate_yaw);
          }
        }
      }
      next_target = current_pose * local_next_target;
      trackingStanley(next_target, current_pose, target_v, target_w, result);
    } else {
      // stanley闭环控制
      trackingStanley(next_target, current_pose, target_v, target_w, result);
      last_ctrl_mode_ = STANLEY_;
    }

    if (edge_ctrl_flag) {
      result.v_velocity = result.v_velocity > 0.2 ? 0.2 : result.v_velocity;
      if (result.w_velocity < -0.35) {
        result.w_velocity = -0.35;
        LOG(WARNING) << "edge result.w_velocity too large";
      }
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
  LOG(INFO) << "******  once stanley  ******";
}

double StanleyController::getRemainCtrlPathLength() {
  std::lock_guard<std::mutex> lock(path_mutex_);
  if (local_path_ == nullptr) {
    return 0.0;
  }
  double dist = local_path_->wpis.back().path_length -
                local_path_->wpis[last_target_index_].path_length;
  LOG(INFO) << "get remain ctrl path length: " << dist;
  return dist;
}

size_t StanleyController::getReferIndex() {
  return last_target_index_;
}

void StanleyController::trackingStanley(const Pose2d &traget_pos,
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
  double steer_angle = 1.5 * dQ + atan(kp_e_ * dYe / (kp_soft_ + target_v)) +
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

void StanleyController::getParams() {
  if (!ptr_navigation_mediator_->isParameterReady()) {
    LOG(ERROR) << "Is mediator's Parameter initialized? ";
    return;
  }

  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.stanley.kag", kag_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.stanley.kp_e", kp_e_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.stanley.kp_soft", kp_soft_,
      1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.stanley.kd_yaw", kd_yaw_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.stanley.kp_v", kp_v_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.stanley.car_length",
      car_length_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.stanley.max_w", max_w_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.stanley.max_vel", max_vel_,
      1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.stanley.min_w", min_w_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.stanley.min_vel", min_vel_,
      1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.stanley.sensor", edge_sensor_,
      std::string("LIDAR"));
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.stanley.psd_forward_install",
      psd_forward_install_, 0.3);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.stanley.psd_back_install",
      psd_back_install_, -0.2);
  LOG(INFO) << "read stanley params: ";
  LOG(INFO) << "kag_: " << kag_ << " kp_e_: " << kp_e_
            << " kp_soft_: " << kp_soft_ << " kd_yaw_: " << kd_yaw_
            << " kp_v_: " << kp_v_;
  LOG(INFO) << "car_length: " << car_length_;
  LOG(INFO) << "max_vel: " << max_vel_ << " max_w:" << max_w_
            << " min_vel: " << min_vel_ << " min_w: " << min_w_;
  LOG(INFO) << "edge_sensor: " << edge_sensor_;
  LOG(INFO) << "psd_install: " << psd_forward_install_ << ", "
            << psd_back_install_;

  setMoveSpeedRange(0.0, max_vel_, 0.0, 0.0, 0.0, max_w_);
  u_steer_angle_ = 0.45 * M_PI;
  l_steer_angle_ = -0.45 * M_PI;
  last_steer_angle_ = 0.0;
}

double StanleyController::checkLimitSteerAngle(double steer_angle) const {
  if (steer_angle > 0) {
    return steer_angle < u_steer_angle_ ? steer_angle : u_steer_angle_;
  } else {
    return steer_angle > l_steer_angle_ ? steer_angle : l_steer_angle_;
  }
}

bool StanleyController::setPath(std::shared_ptr<SubPath> path,
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

bool StanleyController::findTrackTarget(const Pose2d &c_pose,
                                        size_t &path_point_index,
                                        size_t &last_point_index,
                                        double front_distance,
                                        double sum_dist) {
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

TrackCtrl StanleyController::changeCtrlOutput(double vel, double steer_angle_) {
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

double StanleyController::changeToSteerAngle(double vel, double w) {
  return atan(w * car_length_ / vel);
}

void StanleyController::checkMaxVel(double &v_vel, double &w_vel) const {
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

void StanleyController::checkMinVel(double &v_vel, double &w_vel) const {
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
