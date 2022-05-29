/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file path_optimizer.cpp
 *
 *@brief 封装 B样条曲线 优化路径的类
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@version Navigation-v2.0
 *@data 2021-01-06
 ************************************************************************/

#include "bsline/path_optimizer.hpp"
#include <glog/logging.h>
#include "chrono"

#define MINEST_OPTIMIZE_SIZE 2  // 优化路径的最少数量，后续待降低该值下限

namespace CVTE_BABOT {

PathOptimizer::PathOptimizer() {}

void PathOptimizer::showMeParams(const OptPathParams &params) {
  LOG(INFO) << "Params: "
            << " v_limit: " << params.v_limit << "  w_limit: " << params.w_limit
            << " acc_limit: " << params.acc_limit;
  LOG(INFO) << "bsp_delta_time: " << params.bsp_delta_t
            << "  sample_delta_time: " << params.sample_delta_t;
  LOG(INFO) << "start vel: " << params.start_vel
            << "  end_vel: " << params.end_vel;
  LOG(INFO) << "down sample: " << params.down_sample
            << " need_vel_limit: " << params.need_vel_limit
            << " need_opt_ctrl: " << params.need_opt_ctrl;
  LOG(INFO) << "lambda1: " << params.ctrl_point_opt_params.lambda1
            << " lambda2: " << params.ctrl_point_opt_params.lambda2
            << " lambda3: " << params.ctrl_point_opt_params.lambda3
            << " lambda4: " << params.ctrl_point_opt_params.lambda4;
  LOG(INFO) << "use_endpt_opt: " << params.ctrl_point_opt_params.use_endpt_opt;
  LOG(INFO) << "dist: " << params.ctrl_point_opt_params.dist0
            << " min_radiu: " << params.ctrl_point_opt_params.min_rot_radiu;
  LOG(INFO) << "max_iter_nums: "
            << params.ctrl_point_opt_params.max_iteration_num
            << " max_iter_time: "
            << params.ctrl_point_opt_params.max_iteration_time;
}

bool PathOptimizer::judgeSegmentPath(const SubPath &target_path,
                                     std::vector<SubPath> &segment_result) {
  if (target_path.wpis.empty()) {
    return false;  // 如果路径中没有速度值，则认为是不分段的路径（目前是根据速度方向变化来切割）
  }

  std::vector<int> dir_vec;  // 记录转向的索引点
  for (size_t index = 0; index < target_path.wpis.size() - 1; index++) {
    if (target_path.wpis[index].v * target_path.wpis[index + 1].v < 0) {
      dir_vec.push_back(index);
    }
  }

  if (dir_vec.empty()) {
    LOG(INFO) << "No turn.";
    return false;  // 如果转向索引为空，代表路径不存在分段情况
  }

  dir_vec.push_back(target_path.size() - 1);

  int search = 0;
  for (auto dex : dir_vec) {
    SubPath tmp_sub_path;
    for (; search <= dex; ++search) {
      tmp_sub_path.wps.push_back(target_path.wps.at(search));
      tmp_sub_path.wpis.push_back(target_path.wpis[search]);
    }
    if (dex < target_path.size() - 1) {
      tmp_sub_path.wps.push_back(target_path.wps.at(dex + 1));
      tmp_sub_path.wpis.push_back(target_path.wpis[dex + 1]);
    }
    segment_result.push_back(tmp_sub_path);
    // search = dex;
  }
}

void PathOptimizer::recomputePathPointAngle(const SubPath &target_path,
                                            SubPath &optimize_result) {
  // 从新计算一次优化结果中每个点的角度值
  // 因为优化结果的角度值是通过速度值计算，存在反折的情况，改为与下一点的朝向值
  if (optimize_result.empty()) {
    return;
  }

  double yaw = 0.0;
  size_t opti_length = optimize_result.size() - 1;
  size_t orgi_length = target_path.size() - 1;
  // 起点和终点的角度值保持与输入路径的角度值一致
  optimize_result.wps[0].setYaw(target_path.wps[0].getYaw());
  optimize_result.wps[opti_length].setYaw(
      target_path.wps[orgi_length].getYaw());

  for (size_t i = 1; i < opti_length; ++i) {
    yaw = atan2((optimize_result[i + 1].getY() - optimize_result[i].getY()),
                (optimize_result[i + 1].getX() - optimize_result[i].getX()));
    optimize_result[i].setYaw(yaw);
  }
}

SubPath PathOptimizer::curveFitPath(
    const SubPath &target_path, const OptPathParams &params,
    const std::shared_ptr<Costmap2d> &costmap_ptr) {
  SubPath opt_path;
  LOG(INFO) << "curve fit paht size: " << target_path.wps.size();
  showMeParams(params);
  if (target_path.wps.empty()) {
    return opt_path;
  }
  // 当输入路径的点数少于拟合最少值时，暂时先直接返回固定速度值
  if (target_path.wps.size() < MINEST_OPTIMIZE_SIZE) {
    for (int i = 0; i < target_path.wps.size(); ++i) {
      opt_path.wps.push_back(target_path.wps[i]);
      opt_path.wpis.push_back(WayPointInfo(0.2, 0.2, 0.0));
    }
    return opt_path;
  }

  std::vector<Eigen::Vector3d> path_data =
      preparePathData(target_path, params.down_sample);
  Eigen::MatrixXd ctrl_ptr =
      calcCtrolPoints(path_data, params.need_vel_limit, params.bsp_delta_t,
                      params.start_vel, params.end_vel);

  if (params.need_opt_ctrl) {
    if (costmap_ptr == nullptr) {  // 只有需要优化路径的时候，才需要costmap参数
      LOG(WARNING) << "Can't optimize path without costmap.";
    } else {
      LOG(INFO) << "optimize ctrl points";
      auto befort_opt = std::chrono::system_clock::now();
      ctrl_ptr = optimizeCtrlPts(ctrl_ptr, params.bsp_delta_t, costmap_ptr,
                                 params.ctrl_point_opt_params);
      auto aft_opt = std::chrono::system_clock::now();
      std::chrono::duration<double> diff = aft_opt - befort_opt;
      LOG(INFO) << "ctrl points opt time: " << diff.count();
    }
  }
  return calcPathPoseAndVel(ctrl_ptr, params.v_limit, params.w_limit,
                            params.acc_limit, params.bsp_delta_t,
                            params.sample_delta_t);
  // recomputePathPointAngle(target_path, optimize_path);
  // return optimize_path;
}

Eigen::MatrixXd PathOptimizer::calcCtrolPoints(
    std::vector<Eigen::Vector3d> bspline_path, bool need_vel_limit,
    double bsp_delta_t, double start_vel, double end_vel) {
  Eigen::MatrixXd ctrl_pts(bspline_path.size() + 2, 3);
  if (!need_vel_limit) {
    // LOG(ERROR) << "first point before opt: " << bspline_path[0].transpose();
    // LOG(ERROR) << "last point before opt: " <<
    // bspline_path[bspline_path.size() - 1].transpose();
    Eigen::Vector3d first_control_point = 2 * bspline_path[0] - bspline_path[1];
    ctrl_pts(0, 0) = first_control_point(0);
    ctrl_pts(0, 1) = first_control_point(1);
    ctrl_pts(0, 2) = 0;
    for (size_t i = 0; i < bspline_path.size(); ++i) {
      // LOG(INFO) << "ctrl points: " << bspline_path[i].transpose();
      ctrl_pts(i + 1, 0) = bspline_path[i](0);
      ctrl_pts(i + 1, 1) = bspline_path[i](1);
      ctrl_pts(i + 1, 2) = 0;
    }
    Eigen::Vector3d last_control_point =
        2 * bspline_path[bspline_path.size() - 1] -
        bspline_path[bspline_path.size() - 2];
    ctrl_pts(bspline_path.size() + 1, 0) = last_control_point(0);
    ctrl_pts(bspline_path.size() + 1, 1) = last_control_point(1);
    ctrl_pts(bspline_path.size() + 1, 2) = 0;
  } else {
    Eigen::Vector3d start_v;
    start_v(0) = start_vel * cos(bspline_path[0](2));
    start_v(1) = start_vel * sin(bspline_path[0](2));
    start_v(2) = 0.0;

    Eigen::Vector3d end_v;
    end_v(0) = end_vel * cos(bspline_path[bspline_path.size() - 1](2));
    end_v(1) = end_vel * sin(bspline_path[bspline_path.size() - 1](2));
    end_v(2) = 0.0;

    vector<Eigen::Vector3d> start_end_derivatives;
    start_end_derivatives.push_back(start_v);
    start_end_derivatives.push_back(end_v);
    Eigen::Vector3d start_acc(0, 0, 0);
    Eigen::Vector3d end_acc(0, 0, 0);
    start_end_derivatives.push_back(start_acc);
    start_end_derivatives.push_back(end_acc);

    NonUniformBspline::parameterizeToBspline(bsp_delta_t, bspline_path,
                                             start_end_derivatives, ctrl_pts);
  }
  return ctrl_pts;
}

SubPath PathOptimizer::calcPathPoseAndVel(const Eigen::MatrixXd &ctrl_pts,
                                          double v_limit, double w_limit,
                                          double acc_limit, double bsp_delta_t,
                                          double delta_t) {
  SubPath path_result;
  NonUniformBspline pos = NonUniformBspline(ctrl_pts, 3, bsp_delta_t);
  pos.setPhysicalLimits(v_limit, w_limit, acc_limit);
  auto now = std::chrono::system_clock::now();
  pos.reallocateTime(true);
  // auto after_reallocate_time = std::chrono::system_clock::now();
  double tm = 0.0, tmp = 0.0;
  pos.getTimeSpan(tm, tmp);

  path_time = tmp - tm;
  NonUniformBspline bspline_vel = pos.getDerivative();
  Eigen::Vector3d last_pt = pos.evaluateDeBoor(0);
  WayPointInfo last_pt_info;
  last_pt_info.path_length = 0.0;
  double inv_delta_t = 1.0 / delta_t;
  Eigen::Vector3d last_vel = bspline_vel.evaluateDeBoor(tm);
  double last_yaw = atan2(last_vel(1), last_vel(0));
  path_result.wps.reserve(static_cast<int>(path_time / delta_t));
  path_result.wpis.reserve(static_cast<int>(path_time / delta_t));
  for (double t = tm; t <= tmp; t += delta_t) {
    Eigen::Vector3d pt = pos.evaluateDeBoor(t);
    Eigen::Vector3d vel = bspline_vel.evaluateDeBoor(t);
    // Eigen::Vector3d next_vel = bspline_vel.evaluateDeBoor(t + delta_t);
    double yaw = atan2(vel(1), vel(0));
    // double next_yaw = atan2(next_vel(1), next_vel(0));
    pt(2) = yaw;
    path_result.wps.emplace_back(Pose2d(pt(0), pt(1), pt(2)));
    WayPointInfo wpi;
    wpi.v = std::sqrt(vel(0) * vel(0) + vel(1) * vel(1));
    wpi.w = AngleCalculate::angle_diff(yaw, last_yaw) * inv_delta_t;
    wpi.path_length = last_pt_info.path_length +
                      (pt.block<2, 1>(0, 0) - last_pt.block<2, 1>(0, 0)).norm();
    last_pt = pt;
    last_pt_info = wpi;
    last_yaw = yaw;
    path_result.wpis.emplace_back(wpi);
  }
  auto after_calcu_vel = std::chrono::system_clock::now();
  // std::chrono::duration<double> diff1 = after_reallocate_time - now;
  std::chrono::duration<double> diff2 = after_calcu_vel - now;
  // LOG(INFO) << "span time: " << path_time;
  // LOG(INFO) << "time cost of reallocate time: " << diff1.count();
  LOG(INFO) << "time cost of calcu vel: " << diff2.count();
  // LOG(ERROR) << "first point after opt: " << path_result.wps[0].getX() << " "
  // << path_result.wps[0].getY() <<
  //   " " << path_result.wps[0].getYaw();
  // LOG(ERROR) << "last point after opt: " <<
  // path_result.wps[path_result.wps.size() - 1].getX() << " " <<
  // path_result.wps[path_result.wps.size() - 1].getY() <<
  //   " " << path_result.wps[path_result.wps.size() - 1].getYaw();
  return path_result;
}

Eigen::MatrixXd PathOptimizer::optimizeCtrlPts(
    const Eigen::MatrixXd &ctrl_pts, double bsp_delta_t,
    const std::shared_ptr<Costmap2d> &costmap_ptr,
    const CtrlPointOptParams &ctrl_pt_opt_params) {
  int cost_function = BsplineOptimizer::NORMAL_PHASE;
  if (ctrl_pt_opt_params.use_endpt_opt) {
    LOG(INFO) << "use_endpt_opt";
    cost_function = cost_function | BsplineOptimizer::ENDPOINT;
  }
  BsplineOptimizer bspline_opt;
  bspline_opt.setEnvironment(costmap_ptr);
  bspline_opt.setParam(ctrl_pt_opt_params.lambda1, ctrl_pt_opt_params.lambda2,
                       ctrl_pt_opt_params.lambda3, ctrl_pt_opt_params.lambda4,
                       ctrl_pt_opt_params.dist0,
                       ctrl_pt_opt_params.min_rot_radiu,
                       ctrl_pt_opt_params.max_iteration_num,
                       ctrl_pt_opt_params.max_iteration_time);
  return bspline_opt.BsplineOptimizeTraj(ctrl_pts, bsp_delta_t, cost_function,
                                         1, 1);
}

std::vector<Eigen::Vector3d> PathOptimizer::preparePathData(
    const SubPath &target_path, int down_sample_size) {
  // 作用一：转换数据类型
  // 作用二：将路径的起点和终点重复两次，避免优化后不经过起点和终点
  // 作用三：路径降采样，与规划路径的密集程度相关
  std::vector<Eigen::Vector3d> path_result;
  // path_result.push_back(start_point);
  // path_result.push_back(start_point);
  //路径点太少不对路径进行采样
  if (target_path.wps.size() <= MINEST_OPTIMIZE_SIZE) {
    down_sample_size = 1;
  } else {
    down_sample_size = down_sample_size + 1;
  }
  int path_size = target_path.wps.size() - 1;
  for (int i = 0; i < path_size; i += down_sample_size) {
    Eigen::Vector3d point_data(target_path.wps[i].getX(),
                               target_path.wps[i].getY(),
                               target_path.wps[i].getYaw());
    path_result.push_back(point_data);
  }

  Eigen::Vector3d end_point(target_path.wps[path_size].getX(),
                            target_path.wps[path_size].getY(),
                            target_path.wps[path_size].getYaw());
  // auto last_end_point_it = path_result.back();
  // Eigen::Vector3d diff = end_point - last_end_point_it;
  // LOG(INFO) << "diff: " << diff.transpose();
  // if (sqrt(diff(0) * diff(0) + diff(1) * diff(1)) < 0.25) {
  //   LOG(INFO) << "erase last point";
  //   auto end_it = path_result.end();
  //   --end_it;
  //   path_result.erase(end_it);
  // }
  path_result.push_back(end_point);
  // path_result.push_back(end_point);

  return path_result;
}
}  // namespace CVTE_BABOT
