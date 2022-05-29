/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file path_optimizer.hpp
 *
 *@brief 封装 B样条曲线 优化路径的类
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@version Navigation-v2.0
 *@data 2021-01-06
 ************************************************************************/

#ifndef __PATH_OPTIMIZER_HPP
#define __PATH_OPTIMIZER_HPP

#include <pose2d/pose2d.hpp>
#include <vector>
#include "bsline/bsline_base/non_uniform_bspline.h"
#include "bsline/bsline_optimize/bspline_optimizer.h"
#include "path.hpp"

namespace CVTE_BABOT {

struct CtrlPointOptParams {
  CtrlPointOptParams() {}
  CtrlPointOptParams(double k1, double k2, double k3, double k4, double dist,
                     double rot_radiu, int max_iter_num, double max_iter_time,
                     bool endpt_opt) {
    this->lambda1 = k1;
    this->lambda2 = k2;
    this->lambda3 = k3;
    this->lambda4 = k4;
    this->dist0 = dist;
    this->min_rot_radiu = rot_radiu;
    this->max_iteration_num = max_iter_num;
    this->max_iteration_time = max_iter_time;
    this->use_endpt_opt = endpt_opt;
  }
  double lambda1 = 1.0;
  double lambda2 = 10.0;
  double lambda3 = 10;
  double lambda4 = 100;
  double dist0 = 1.5;
  double min_rot_radiu = 0.20;
  int max_iteration_num = 10;
  double max_iteration_time = 0.1;
  bool use_endpt_opt;
};

struct OptPathParams {
  OptPathParams() {}

  OptPathParams(double v_lit, double w_lit, double a_lit, double bsp_del_t,
                double s_del_t, double start_v, double end_v, int d_sample,
                bool bsp_vel_lit, bool opt_ctrl,
                const CtrlPointOptParams &opt_params) {
    this->v_limit = v_lit;
    this->w_limit = w_lit;
    this->acc_limit = a_lit;
    this->bsp_delta_t = bsp_del_t;
    this->sample_delta_t = s_del_t;
    this->start_vel = start_v;
    this->end_vel = end_v;
    this->down_sample = d_sample;
    this->need_vel_limit = bsp_vel_lit;
    this->need_opt_ctrl = opt_ctrl;
    this->ctrl_point_opt_params = opt_params;
  }

  double v_limit = 0.0;
  double w_limit = 0.0;
  double acc_limit = 0.0;
  double bsp_delta_t = 0.0;
  double sample_delta_t = 0.0;
  double start_vel = 0.0;
  double end_vel = 0.0;
  int down_sample = 0;
  bool need_vel_limit = false;
  bool need_opt_ctrl = false;

  CtrlPointOptParams ctrl_point_opt_params;
};

class PathOptimizer {
 public:
  PathOptimizer();
  ~PathOptimizer() = default;

  bool judgeSegmentPath(const SubPath &target_path,
                        std::vector<SubPath> &segment_result);

  SubPath curveFitPath(const SubPath &target_path, const OptPathParams &params,
                       const std::shared_ptr<Costmap2d> &costmap_ptr = nullptr);

  double getPathTime() { return path_time; };

 protected:
  void showMeParams(const OptPathParams &params);

  Eigen::MatrixXd calcCtrolPoints(std::vector<Eigen::Vector3d> bspline_path,
                                  bool need_vel_limit, double bsp_delta_t,
                                  double start_vel, double end_vel);

  SubPath calcPathPoseAndVel(const Eigen::MatrixXd &ctrl_pts, double v_limit,
                             double w_limit, double acc_limit,
                             double bsp_delta_t, double delta_t);

  Eigen::MatrixXd optimizeCtrlPts(const Eigen::MatrixXd &ctrl_pts,
                                  double bsp_delta_t,
                                  const std::shared_ptr<Costmap2d> &costmap_ptr,
                                  const CtrlPointOptParams &ctrl_pt_opt_params);

  std::vector<Eigen::Vector3d> preparePathData(const SubPath &target_path,
                                               int down_sample_size);

  void recomputePathPointAngle(const SubPath &target_path,
                               SubPath &optimize_result);

 private:
  double path_time = 0;
};
}  // namespace CVTE_BABOT

#endif  // end of __PATH_OPTIMIZER_HPP