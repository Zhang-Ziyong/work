/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file bspline_optimizer.cpp
 *
 *@brief 路径优化库
 *
 *@modified by linyanlong(linyanlong@cvte.com)
 *
 *@version Navigation-v2.0
 *@data 2021-01-06
 ************************************************************************/

#include "bsline/bsline_optimize/bspline_optimizer.h"
#include "costmap_2d.hpp"

#include <glog/logging.h>
#include <nlopt.hpp>
// using namespace std;

namespace CVTE_BABOT {

const int BsplineOptimizer::SMOOTHNESS = (1 << 0);
const int BsplineOptimizer::DISTANCE = (1 << 1);
const int BsplineOptimizer::FEASIBILITY = (1 << 2);
const int BsplineOptimizer::ENDPOINT = (1 << 3);
const int BsplineOptimizer::GUIDE = (1 << 4);
const int BsplineOptimizer::WAYPOINTS = (1 << 6);

const int BsplineOptimizer::GUIDE_PHASE =
    BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::GUIDE;
const int BsplineOptimizer::NORMAL_PHASE = BsplineOptimizer::SMOOTHNESS |
                                           BsplineOptimizer::DISTANCE |
                                           BsplineOptimizer::FEASIBILITY;

// const int BsplineOptimizer::NORMAL_PHASE = BsplineOptimizer::SMOOTHNESS |
// BsplineOptimizer::DISTANCE;

void BsplineOptimizer::setParam(double lambda1, double lambda2, double lambda3,
                                double lambda4, double dist,
                                double min_rot_radiu, int max_iter_nums,
                                double max_iter_time) {
  lambda1_ = lambda1;
  lambda2_ = lambda2;
  lambda3_ = lambda3;
  lambda4_ = lambda4;
  dist0_ = dist;
  min_rot_radiu_ = min_rot_radiu;
  max_iteration_num_[0] = max_iter_nums;
  max_iteration_num_[1] = max_iter_nums;
  max_iteration_num_[2] = max_iter_nums;
  max_iteration_num_[3] = max_iter_nums;
  max_iteration_time_[0] = max_iter_time;
  max_iteration_time_[1] = max_iter_time;
  max_iteration_time_[2] = max_iter_time;
  max_iteration_time_[3] = max_iter_time;
  algorithm1_ = 15;
  algorithm2_ = 11;
  // lambda1_ = 1.0;
  // lambda2_ = 10.0;
  // lambda3_ = 10;
  // lambda4_ = 100;
  // // lambda7_ = 100.0;
  // dist0_ = 1.5;
  // min_rot_radiu_ = 0.20;
  // algorithm1_ = 15;
  // algorithm2_ = 11;
  // max_iteration_num_[0] = 10;
  // max_iteration_num_[1] = 10;
  // max_iteration_num_[2] = 10;
  // max_iteration_num_[3] = 10;
  // max_iteration_time_[0] = 0.05;
  // max_iteration_time_[1] = 0.05;
  // max_iteration_time_[2] = 0.03;
  // max_iteration_time_[3] = 0.03;
  order_ = 3;
}

void BsplineOptimizer::setEnvironment(const std::shared_ptr<Costmap2d> &env) {
  this->ptr_costmap_ = env;
  resolution_ = env->getResolution();
  resolution_inv_ = 1 / env->getResolution();
}

void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points) {
  control_points_ = points;
  dim_ = control_points_.cols();
}

void BsplineOptimizer::setBsplineInterval(const double &ts) {
  bspline_interval_ = ts;
}

void BsplineOptimizer::setTerminateCond(const int &max_num_id,
                                        const int &max_time_id) {
  max_num_id_ = max_num_id;
  max_time_id_ = max_time_id;
}

void BsplineOptimizer::setCostFunction(const int &cost_code) {
  cost_function_ = cost_code;

  // print optimized cost function
  string cost_str;
  if (cost_function_ & SMOOTHNESS) {
    cost_str += "smooth |";
  }
  if (cost_function_ & DISTANCE) {
    if (ptr_costmap_) {
      cost_str += " dist  |";
    }
  }
  if (cost_function_ & FEASIBILITY) {
    cost_str += " feasi |";
  }
  if (cost_function_ & ENDPOINT) {
    cost_str += " endpt |";
  }
  if (cost_function_ & GUIDE) {
    cost_str += " guide |";
  }
  if (cost_function_ & WAYPOINTS) {
    cost_str += " waypt |";
  }

  LOG(INFO) << "cost func: " << cost_str;
}

void BsplineOptimizer::setGuidePath(const vector<Eigen::Vector3d> &guide_pt) {
  guide_pts_ = guide_pt;
}

void BsplineOptimizer::setWaypoints(const vector<Eigen::Vector3d> &waypts,
                                    const vector<int> &waypt_idx) {
  waypoints_ = waypts;
  waypt_idx_ = waypt_idx;
}

Eigen::MatrixXd BsplineOptimizer::BsplineOptimizeTraj(
    const Eigen::MatrixXd &points, const double &ts, const int &cost_function,
    int max_num_id, int max_time_id) {
  setControlPoints(points);
  setBsplineInterval(ts);
  setCostFunction(cost_function);
  setTerminateCond(max_num_id, max_time_id);

  optimize();
  return this->control_points_;
}

void BsplineOptimizer::optimize() {
  /* initialize solver */
  iter_num_ = 0;
  min_cost_ = std::numeric_limits<double>::max();
  const int pt_num = control_points_.rows();
  g_q_.resize(pt_num);
  g_smoothness_.resize(pt_num);
  g_distance_.resize(pt_num);
  g_feasibility_.resize(pt_num);
  g_endpoint_.resize(pt_num);
  g_waypoints_.resize(pt_num);
  g_guide_.resize(pt_num);

  if (cost_function_ & ENDPOINT) {
    variable_num_ = dim_ * (pt_num - order_);
    // end position used for hard constraint
    end_pt_ = (1 / 6.0) * (control_points_.row(pt_num - 3) +
                           4 * control_points_.row(pt_num - 2) +
                           control_points_.row(pt_num - 1));
    end_dir_ =
        (control_points_.row(pt_num - 1) - control_points_.row(pt_num - 3));
  } else {
    variable_num_ = max(0, dim_ * (pt_num - 2 * order_));
  }

  /* do optimization using NLopt slover */
  nlopt::opt opt(nlopt::algorithm(isQuadratic() ? algorithm1_ : algorithm2_),
                 variable_num_);
  opt.set_min_objective(BsplineOptimizer::costFunction, this);
  opt.set_maxeval(max_iteration_num_[max_num_id_]);
  opt.set_maxtime(max_iteration_time_[max_time_id_]);
  opt.set_xtol_rel(1e-5);

  vector<double> q(variable_num_);
  for (int i = order_; i < pt_num; ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_)
      continue;
    for (int j = 0; j < dim_; j++) {
      q[dim_ * (i - order_) + j] = control_points_(i, j);
    }
  }

  if (dim_ != 1) {
    vector<double> lb(variable_num_), ub(variable_num_);
    const double bound = 10.0;
    for (int i = 0; i < variable_num_; ++i) {
      lb[i] = q[i] - bound;
      ub[i] = q[i] + bound;
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
  }

  try {
    // cout << fixed << setprecision(7);
    // vec_time_.clear();
    // vec_cost_.clear();
    // time_start_ = ros::Time::now();

    double final_cost;
    nlopt::result result = opt.optimize(q, final_cost);

    /* retrieve the optimization result */
    // cout << "Min cost:" << min_cost_ << endl;
  } catch (std::exception &e) {
    LOG(ERROR) << "[Optimization]: nlopt exception";
    LOG(ERROR) << e.what() << endl;
  }

  for (int i = order_; i < control_points_.rows(); ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_)
      continue;
    for (int j = 0; j < dim_; j++) {
      control_points_(i, j) = best_variable_[dim_ * (i - order_) + j];
    }
  }

  if (!(cost_function_ & GUIDE)) {
    LOG(INFO) << "iter num: " << iter_num_;
  }
}

void BsplineOptimizer::calcSmoothnessCost(const vector<Eigen::Vector3d> &q,
                                          double &cost,
                                          vector<Eigen::Vector3d> &gradient) {
  double ts = bspline_interval_;
  double ts_inv3 = 1 / ts / ts / ts;
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);
  Eigen::Vector3d jerk, temp_j;

  for (int i = order_; i < q.size() - order_; i++) {
    /* evaluate jerk */
    jerk = (q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i]) * ts_inv3;
    // LOG(INFO) << "jeck: " << jerk.transpose();
    cost += jerk.squaredNorm();
    temp_j = 2.0 * jerk;
    /* jerk gradient */
    gradient[i + 0] += -temp_j * ts_inv3;
    gradient[i + 1] += 3.0 * temp_j * ts_inv3;
    gradient[i + 2] += -3.0 * temp_j * ts_inv3;
    gradient[i + 3] += temp_j * ts_inv3;
  }
  // LOG(ERROR) << std::endl;
}

void BsplineOptimizer::getSurroundPts(const Eigen::Vector3d &pos,
                                      Eigen::Vector3d pts[2][2][2],
                                      Eigen::Vector3d &diff) {
  // if (!ptr_costmap_->worldTomap(pos(0), )) {
  //   cout << "pos invalid for interpolation." << endl;
  // }

  /* interpolation position */
  Eigen::Vector3d pos_m = pos - 0.5 * resolution_ * Eigen::Vector3d::Ones();
  int i_x = 0, i_y = 0;
  ptr_costmap_->worldToMapEnforceBounds(pos_m(0), pos_m(1), i_x, i_y);

  WorldmapPoint wm_point;
  ptr_costmap_->mapToWorld({i_x, i_y}, wm_point);
  Eigen::Vector3d idx_pos(wm_point.d_x, wm_point.d_y, pos(2));
  diff = (pos - idx_pos) * resolution_inv_;

  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        CostmapPoint current_idx = {i_x + x, i_y + y};
        WorldmapPoint current_point;
        ptr_costmap_->mapToWorld(current_idx, current_point);

        Eigen::Vector3d current_pos(current_point.d_x, current_point.d_y,
                                    pos(2));
        pts[x][y][z] = current_pos;
      }
    }
  }
}

void BsplineOptimizer::interpolateTrilinear(double values[2][2][2],
                                            const Eigen::Vector3d &diff,
                                            double &value,
                                            Eigen::Vector3d &grad) {
  // trilinear interpolation
  double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0];
  double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1];
  double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0];
  double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1];
  double v0 = (1 - diff(1)) * v00 + diff(1) * v10;
  double v1 = (1 - diff(1)) * v01 + diff(1) * v11;

  value = (1 - diff(2)) * v0 + diff(2) * v1;

  grad[2] = (v1 - v0) * resolution_inv_;
  grad[1] =
      ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  grad[0] *= resolution_inv_;
}

void BsplineOptimizer::getSurroundDistance(Eigen::Vector3d pts[2][2][2],
                                           double dists[2][2][2]) {
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        dists[x][y][z] = getDistance(pts[x][y][z]);
      }
    }
  }
}

double BsplineOptimizer::getDistance(const Eigen::Vector3d &pos) {
  // Eigen::Vector3i id;
  // posToIndex(pos, id);
  // boundIndex(id);
  int i_x = 0, i_y = 0;
  ptr_costmap_->worldToMapEnforceBounds(pos(0), pos(1), i_x, i_y);

  auto cost = ptr_costmap_->getCost(i_x, i_y);
  // if (cost == NO_INFORMATION) {
  //   return 0.0;
  // } else if (cost == LETHAL_OBSTACLE) {
  //   return 0.1;
  // } else if (cost == INSCRIBED_INFLATED_OBSTACLE) {
  //   return 0.3;
  // } else if (cost != FREE_SPACE) {
  //   double dis = 0.3 - log(cost / 252.0) / 3.0;
  //   // LOG(ERROR) << (int) cost << ", dis " << dis;
  //   return dis;
  // } else {
  //   return 1.1;
  // }
  return cost;
}

void BsplineOptimizer::evaluateEDTWithGrad(const Eigen::Vector3d &pos,
                                           double time, double &dist,
                                           Eigen::Vector3d &grad) {
  Eigen::Vector3d diff;
  Eigen::Vector3d sur_pts[2][2][2];
  getSurroundPts(pos, sur_pts, diff);

  double dists[2][2][2];
  getSurroundDistance(sur_pts, dists);
  // for (size_t i = 0; i < 2; i++) {
  //   for (size_t j = 0; j < 2; j++) { std::cout << dists[i][j][0] << " "; }
  //   std::cout << std::endl;
  // }
  // std::cout << std::endl;

  interpolateTrilinear(dists, diff, dist, grad);
  DisGradPoins dis;
  dis.distance = dist;
  dis.grad_x = -grad[0];
  dis.grad_y = -grad[1];
  dis.pos_x = pos[0];
  dis.pos_y = pos[1];
  vec_dgps_.push_back(dis);
}

void BsplineOptimizer::calcDistanceCost(const vector<Eigen::Vector3d> &q,
                                        double &cost,
                                        vector<Eigen::Vector3d> &gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  double dist;
  Eigen::Vector3d dist_grad, g_zero(0, 0, 0);

  int end_idx = (cost_function_ & ENDPOINT) ? q.size() : q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    evaluateEDTWithGrad(q[i], -1.0, dist, dist_grad);
    if (dist_grad.norm() > 1e-4)
      dist_grad.normalize();

    if (dist > dist0_) {
      // cost += 1 / (dist * dist);
      // gradient[i] += -2 * dist_grad / (dist * dist * dist);
      // cost += pow(dist - dist0_, 2);
      cost += fabs(dist - dist0_);
      // gradient[i] += 2.0 * (dist - dist0_) * dist_grad;
      gradient[i] += dist_grad;
    }
  }
  LOG(INFO) << "dist cost: " << cost;
}

void BsplineOptimizer::calcFeasibilityCost(const vector<Eigen::Vector3d> &q,
                                           double &cost,
                                           vector<Eigen::Vector3d> &gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  /* abbreviation */
  double ts, ts_inv, ts_inv2, ts_inv4;

  ts = bspline_interval_;
  ts_inv = 1.0 / ts;
  ts_inv2 = 1 / ts / ts;
  ts_inv4 = ts_inv2 * ts_inv2;

  /* velocity feasibility */
  // for (int i = 0; i < q.size() - 1; i++) {
  //   Eigen::Vector3d vi = q[i + 1] - q[i];

  //   for (int j = 0; j < 3; j++) {
  //     double vd = vi(j) * vi(j) * ts_inv2 - vm2;
  //     if (vd > 0.0) {
  //       cost += pow(vd, 2);

  //       double temp_v = 4.0 * vd * ts_inv2;
  //       gradient[i + 0](j) += -temp_v * vi(j);
  //       gradient[i + 1](j) += temp_v * vi(j);
  //     }
  //   }
  // }

  // /* acceleration feasibility */
  // for (int i = 0; i < q.size() - 2; i++) {
  //   Eigen::Vector3d ai = q[i + 2] - 2 * q[i + 1] + q[i];

  //   for (int j = 0; j < 3; j++) {
  //     double ad = ai(j) * ai(j) * ts_inv4 - am2;
  //     if (ad > 0.0) {
  //       cost += pow(ad, 2);

  //       double temp_a = 4.0 * ad * ts_inv4;
  //       gradient[i + 0](j) += temp_a * ai(j);
  //       gradient[i + 1](j) += -2 * temp_a * ai(j);
  //       gradient[i + 2](j) += temp_a * ai(j);
  //     }
  //   }
  // }
  // Eigen::Vector3d v0(0.1 * cos(start_pose_(2)), 0.1 * sin(start_pose_(2)),
  // 0);

  // for (int i = 0; i < q.size() - 2; i++) {
  //   Eigen::Vector3d vi = (q[i + 1] - q[i]);
  //   Eigen::Vector3d vi1 = (q[i + 2] - q[i + 1]);
  //   double vi_norm = vi.norm();
  //   double vi1_norm = vi1.norm();
  //   double temp_value = vi_norm*vi_norm*vi1_norm*vi1_norm;
  //   double x = (vi.transpose()*vi1).norm() / (std::sqrt(temp_value));
  //   double rot_angle2 = acos(x) * acos(x);
  //   double min_rot_angle2 = min_rot_angle_ * min_rot_angle_;
  //   if(rot_angle2 > min_rot_angle2) {
  //     double temp_cost = (rot_angle2 - min_rot_angle2) * (rot_angle2 -
  //     min_rot_angle2); cost += temp_cost; double dfdx = 2 * (rot_angle2 -
  //     min_rot_angle2) * 2 * acos(x) * (-1) / (std::sqrt(1 - x*x));
  //     Eigen::Vector3d dxdvi = (vi1 * std::sqrt(temp_value) -
  //     (vi.transpose()*vi1).norm() * 0.5 *
  //      (1/sqrt(temp_value)) *2 * vi * vi1_norm * vi1_norm) / temp_value;
  //     Eigen::Vector3d dxdvi1 = (vi * std::sqrt(temp_value) -
  //     (vi.transpose()*vi1).norm() * 0.5 *
  //      (1/sqrt(temp_value)) *2 * vi1 * vi_norm * vi_norm) / temp_value;
  //     Eigen::Matrix3d dvidqi = -Eigen::Matrix3d::Identity();
  //     Eigen::Matrix3d dvidqi1 = Eigen::Matrix3d::Identity();
  //     Eigen::Matrix3d dvi1dqi1 = -Eigen::Matrix3d::Identity();
  //     Eigen::Matrix3d dvi1dqi2 = Eigen::Matrix3d::Identity();
  //     Eigen::Vector3d dfdqi = dfdx * dvidqi * dxdvi;
  //     Eigen::Vector3d dfdqi1 = dfdx * dvidqi1 * dxdvi + dfdx * dvi1dqi1 *
  //     dxdvi1; Eigen::Vector3d dfdqi2 = dfdx * dvi1dqi2 * dxdvi1; gradient[i]
  //     += dfdqi; gradient[i+1] += dfdqi1; gradient[i+2] += dfdqi2;
  //   }
  for (int i = order_; i < q.size() - order_; i++) {
    Eigen::Vector3d vi = (q[i + 1] - q[i]);
    Eigen::Vector3d vi1 = (q[i + 2] - q[i + 1]);
    double vi_norm = vi.norm();
    double vi1_norm = vi1.norm();
    double temp_value = vi_norm * vi_norm * vi1_norm * vi1_norm;
    double x = (vi.transpose() * vi1).norm() / (std::sqrt(temp_value));
    double rot_angle2 = acos(x);
    double curve = rot_angle2 / vi_norm;
    double max_curve = 1.0 / min_rot_radiu_;
    // double max_curve = 0.1;
    if (curve > max_curve) {
      double temp_cost = (curve - max_curve) * (curve - max_curve);
      cost += temp_cost;
      double dfdx =
          2 * (-1) * (curve - max_curve) / (std::sqrt(1 - x * x)) / vi_norm;
      Eigen::Vector3d dxdvi =
          (vi1 * std::sqrt(temp_value) - (vi.transpose() * vi1).norm() * 0.5 *
                                             (1 / sqrt(temp_value)) * 2 * vi *
                                             vi1_norm * vi1_norm) /
          temp_value;
      Eigen::Vector3d dxdvi1 =
          (vi * std::sqrt(temp_value) - (vi.transpose() * vi1).norm() * 0.5 *
                                            (1 / sqrt(temp_value)) * 2 * vi1 *
                                            vi_norm * vi_norm) /
          temp_value;
      Eigen::Matrix3d dvidqi = -Eigen::Matrix3d::Identity();
      Eigen::Matrix3d dvidqi1 = Eigen::Matrix3d::Identity();
      Eigen::Matrix3d dvi1dqi1 = -Eigen::Matrix3d::Identity();
      Eigen::Matrix3d dvi1dqi2 = Eigen::Matrix3d::Identity();
      Eigen::Vector3d dfdqi = dfdx * dvidqi * dxdvi + 0.5 * rot_angle2 * vi /
                                                          vi_norm / vi_norm /
                                                          vi_norm;
      Eigen::Vector3d dfdqi1 =
          dfdx * dvidqi1 * dxdvi + dfdx * dvi1dqi1 * dxdvi1 -
          0.5 * rot_angle2 * vi / vi_norm / vi_norm / vi_norm;
      Eigen::Vector3d dfdqi2 = dfdx * dvi1dqi2 * dxdvi1;
      gradient[i] += dfdqi;
      gradient[i + 1] += dfdqi1;
      gradient[i + 2] += dfdqi2;
    }
  }

  //  double min_rot_radiu2 = min_rot_radiu_ * min_rot_radiu_;
  //  for (int i = 0; i < q.size() - 2; i++) {
  //   Eigen::Vector3d ai = (q[i + 2] - 2 * q[i + 1] + q[i]) * ts_inv2;
  //   Eigen::Vector3d vi = (q[i + 1] - q[i]) * ts_inv;
  //   double vi_norm = vi.norm();
  //   double ai_norm = ai.norm();
  //   double rot_radiu2 = (1 + vi_norm*vi_norm)*(1 + vi_norm*vi_norm)*(1 +
  //   vi_norm*vi_norm)/(ai_norm*ai_norm); LOG(ERROR) << "rot radius: " <<
  //   rot_radiu2; if(rot_radiu2 < min_rot_radiu2) {
  //     double temp_cost = (rot_radiu2 - min_rot_radiu2) * (rot_radiu2 -
  //     min_rot_radiu2); cost += temp_cost; double dfdr2 = 2 * (temp_cost);
  //     Eigen::Vector3d dr2dvi = 6*(1+vi_norm*vi_norm)*(1+vi_norm*vi_norm)*vi /
  //     (ai_norm*ai_norm); Eigen::Vector3d dr2dai = -2*(1 + vi_norm*vi_norm)*(1
  //     + vi_norm*vi_norm)*(1 + vi_norm*vi_norm)*ai /
  //      (ai_norm * ai_norm * ai_norm * ai_norm);
  //     Eigen::Matrix3d dvidqi = -Eigen::Matrix3d::Identity() * ts_inv;
  //     Eigen::Matrix3d dvidqi1 = Eigen::Matrix3d::Identity() * ts_inv;
  //     Eigen::Matrix3d daidqi = Eigen::Matrix3d::Identity() * ts_inv2;
  //     Eigen::Matrix3d daidqi1 = -2 * Eigen::Matrix3d::Identity() * ts_inv2;
  //     Eigen::Matrix3d daidqi2 = Eigen::Matrix3d::Identity() * ts_inv2;

  //     Eigen::Vector3d dfdqi = dfdr2 * dvidqi * dr2dvi + dfdr2 * daidqi *
  //     dr2dai; Eigen::Vector3d dfdqi1 = dfdr2 * dvidqi1 * dr2dvi + dfdr2 *
  //     daidqi1 * dr2dai; Eigen::Vector3d dfdqi2 = dfdr2 * daidqi2 * dr2dai;
  //     gradient[i] += dfdqi;
  //     gradient[i+1] += dfdqi1;
  //     gradient[i+2] += dfdqi2;
  //   }
  // }
  /*angle velocity feasibility*/
  // double min_cos_cost2 = min_cos_cost_ * min_cos_cost_;
  // for(int i = 0;i < q.size() - 2;i++) {
  //   Eigen::Vector3d vi0 = (q[i + 1] - q[i]) * ts_inv;
  //   Eigen::Vector3d vi1 = (q[i + 2] - q[i+1]) * ts_inv;
  //   double vi0vi1 = (vi0.transpose() * vi1).norm();
  //   double cos_angle = vi0vi1 / (vi0.norm() * vi1.norm());
  //   LOG(ERROR) << "cos_angle: " << cos_angle;
  //   double err = cos_angle * cos_angle - min_cos_cost2;
  //   if(err < 0) {
  //     cost += err * err;
  //     // LOG(ERROR) << "angle vel cost: " << err * err;
  //     double temp = vi0.norm() * vi1.norm() * vi0.norm() * vi1.norm();
  //     Eigen::Vector3d dfdvi0 = 2 * err * (2 * vi0vi1 * vi1 - 2 * vi1.norm() *
  //     vi1.norm() * vi0) /
  //       (temp * temp);
  //     Eigen::Vector3d dfdvi1 = 2 * err * (2 * vi0vi1 * vi0 - 2 * vi0.norm() *
  //     vi0.norm() * vi1) /
  //       (temp * temp);

  //     gradient[i] += -ts_inv * dfdvi0;
  //     gradient[i+1] += ts_inv * dfdvi0 - ts_inv * dfdvi1;
  //     gradient[i+2] += ts_inv * dfdvi1;
  //   }
  // }
}

void BsplineOptimizer::calcEndpointCost(const vector<Eigen::Vector3d> &q,
                                        double &cost,
                                        vector<Eigen::Vector3d> &gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  // zero cost and gradient in hard constraints
  Eigen::Vector3d q_3, q_2, q_1, dq;
  q_3 = q[q.size() - 3];
  q_2 = q[q.size() - 2];
  q_1 = q[q.size() - 1];

  dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - end_pt_;

  Eigen::Vector3d vel = q_3 - q_1;
  Eigen::Vector3d dv = vel - end_dir_;

  cost += dq.squaredNorm() + dv.squaredNorm();

  gradient[q.size() - 3] += 2 * dq * (1 / 6.0) - 2 * dv;
  gradient[q.size() - 2] += 2 * dq * (4 / 6.0);
  gradient[q.size() - 1] += 2 * dq * (1 / 6.0) + 2 * dv;
}

void BsplineOptimizer::calcWaypointsCost(const vector<Eigen::Vector3d> &q,
                                         double &cost,
                                         vector<Eigen::Vector3d> &gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  Eigen::Vector3d q1, q2, q3, dq;

  // for (auto wp : waypoints_) {
  for (int i = 0; i < waypoints_.size(); ++i) {
    Eigen::Vector3d waypt = waypoints_[i];
    int idx = waypt_idx_[i];

    q1 = q[idx];
    q2 = q[idx + 1];
    q3 = q[idx + 2];

    dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
    cost += dq.squaredNorm();

    gradient[idx] += dq * (2.0 / 6.0);      // 2*dq*(1/6)
    gradient[idx + 1] += dq * (8.0 / 6.0);  // 2*dq*(4/6)
    gradient[idx + 2] += dq * (2.0 / 6.0);
  }
}

/* use the uniformly sampled points on a geomertic path to guide the
 * trajectory. For each control points to be optimized, it is assigned a
 * guiding point on the path and the distance between them is penalized */
void BsplineOptimizer::calcGuideCost(const vector<Eigen::Vector3d> &q,
                                     double &cost,
                                     vector<Eigen::Vector3d> &gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  int end_idx = q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    Eigen::Vector3d gpt = guide_pts_[i - order_];
    cost += (q[i] - gpt).squaredNorm();
    gradient[i] += 2 * (q[i] - gpt);
  }
}

void BsplineOptimizer::combineCost(const std::vector<double> &x,
                                   std::vector<double> &grad,
                                   double &f_combine) {
  /* convert the NLopt format vector to control points. */

  // This solver can support 1D-3D B-spline optimization, but we use Vector3d to
  // store each control point For 1D case, the second and third elements are
  // zero, and similar for the 2D case.
  // 起始点
  for (int i = 0; i < order_; i++) {
    for (int j = 0; j < dim_; ++j) { g_q_[i][j] = control_points_(i, j); }
    for (int j = dim_; j < 3; ++j) { g_q_[i][j] = 0.0; }
  }

  // 路径点
  for (int i = 0; i < variable_num_ / dim_; i++) {
    for (int j = 0; j < dim_; ++j) { g_q_[i + order_][j] = x[dim_ * i + j]; }
    for (int j = dim_; j < 3; ++j) { g_q_[i + order_][j] = 0.0; }
  }

  // 终点
  if (!(cost_function_ & ENDPOINT)) {
    for (int i = 0; i < order_; i++) {
      for (int j = 0; j < dim_; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] =
            control_points_(control_points_.rows() - order_ + i, j);
      }
      for (int j = dim_; j < 3; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] = 0.0;
      }
    }
  }

  f_combine = 0.0;
  grad.resize(variable_num_);
  fill(grad.begin(), grad.end(), 0.0);

  /*  evaluate costs and their gradient  */
  double f_smoothness, f_distance, f_feasibility, f_endpoint, f_guide,
      f_waypoints;
  f_smoothness = f_distance = f_feasibility = f_endpoint = f_guide =
      f_waypoints = 0.0;

  if (cost_function_ & SMOOTHNESS) {
    calcSmoothnessCost(g_q_, f_smoothness, g_smoothness_);
    f_combine += lambda1_ * f_smoothness;
    for (int i = 0; i < variable_num_ / dim_; i++) {
      for (int j = 0; j < dim_; j++) {
        grad[dim_ * i + j] += lambda1_ * g_smoothness_[i + order_](j);
      }
    }
  }
  if (cost_function_ & DISTANCE) {
    calcDistanceCost(g_q_, f_distance, g_distance_);
    f_combine += lambda2_ * f_distance;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += lambda2_ * g_distance_[i + order_](j);
  }
  if (cost_function_ & FEASIBILITY) {
    calcFeasibilityCost(g_q_, f_feasibility, g_feasibility_);
    // LOG(INFO) << "cost angle vel: " << f_feasibility;
    f_combine += lambda3_ * f_feasibility;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += lambda3_ * g_feasibility_[i + order_](j);
  }
  if (cost_function_ & ENDPOINT) {
    calcEndpointCost(g_q_, f_endpoint, g_endpoint_);
    f_combine += lambda4_ * f_endpoint;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += lambda4_ * g_endpoint_[i + order_](j);
  }
  if (cost_function_ & GUIDE) {
    calcGuideCost(g_q_, f_guide, g_guide_);
    f_combine += lambda5_ * f_guide;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += lambda5_ * g_guide_[i + order_](j);
  }
  if (cost_function_ & WAYPOINTS) {
    calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
    f_combine += lambda7_ * f_waypoints;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += lambda7_ * g_waypoints_[i + order_](j);
  }
  LOG(INFO) << iter_num_ << ", total cost: " << f_combine;
  /*  print cost  */
  // if ((cost_function_ & WAYPOINTS) && iter_num_ % 10 == 0) {
  //   cout << iter_num_ << ", total: " << f_combine << ", acc: " << lambda8_ *
  //   f_view
  //        << ", waypt: " << lambda7_ * f_waypoints << endl;
  // }

  // if (optimization_phase_ == SECOND_PHASE) {
  //  << ", smooth: " << lambda1_ * f_smoothness
  //  << " , dist:" << lambda2_ * f_distance
  //  << ", fea: " << lambda3_ * f_feasibility << endl;
  // << ", end: " << lambda4_ * f_endpoint
  // << ", guide: " << lambda5_ * f_guide
  // }
}

double BsplineOptimizer::costFunction(const std::vector<double> &x,
                                      std::vector<double> &grad,
                                      void *func_data) {
  BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
  double cost;
  opt->combineCost(x, grad, cost);
  opt->iter_num_++;

  /* save the min cost result */
  if (cost < opt->min_cost_) {
    opt->min_cost_ = cost;
    opt->best_variable_ = x;
  }
  // LOG(ERROR) << "opt cost: " << cost;
  return cost;

  // /* evaluation */
  // ros::Time te1 = ros::Time::now();
  // double time_now = (te1 - opt->time_start_).toSec();
  // opt->vec_time_.push_back(time_now);
  // if (opt->vec_cost_.size() == 0)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else if (opt->vec_cost_.back() > f_combine)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else
  // {
  //   opt->vec_cost_.push_back(opt->vec_cost_.back());
  // }
}

vector<Eigen::Vector3d> BsplineOptimizer::matrixToVectors(
    const Eigen::MatrixXd &ctrl_pts) {
  vector<Eigen::Vector3d> ctrl_q;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    ctrl_q.push_back(ctrl_pts.row(i));
  }
  return ctrl_q;
}

Eigen::MatrixXd BsplineOptimizer::getControlPoints() {
  return this->control_points_;
}

bool BsplineOptimizer::isQuadratic() {
  if (cost_function_ == GUIDE_PHASE) {
    return true;
  } else if (cost_function_ == (SMOOTHNESS | WAYPOINTS)) {
    return true;
  }
  return false;
}

}  // namespace CVTE_BABOT