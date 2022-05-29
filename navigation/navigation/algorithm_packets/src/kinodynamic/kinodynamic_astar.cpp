/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file kinodynamic_astar.cpp
 *
 *@brief 动力学约束的Hybrid A星
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Copyright 2019 Boyu Zhou, Aerial Robotics Group
 * information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
 *@version Navigation-v2.0
 *@data 2020-12-02
 ************************************************************************/

#include "kinodynamic/kinodynamic_astar.hpp"
// #include "pose2d.hpp"
#include <glog/logging.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

namespace CVTE_BABOT {

KinodynamicAstar::KinodynamicAstar() {
  /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++) { path_node_pool_[i] = new PathNode; }
  // setParam();
}

KinodynamicAstar::~KinodynamicAstar() {
  for (int i = 0; i < allocate_num_; i++) { delete path_node_pool_[i]; }
}

void KinodynamicAstar::init(const std::string &planner) {
  if (planner == "global") {
    one_shot_radiu_ = 0.5;
    one_shot_dist_ = 2.0;
    one_shot_inter_dist_ = 0.05;
    angle_res_ = 1.0 / 4.0;
    max_angle_vel_ = 0.3 * M_PI;
    enable_retreat_ = true;
    check_shot_ = false;
  } else {
    one_shot_radiu_ = 0.1;
    one_shot_dist_ = 0.3;
    one_shot_inter_dist_ = 0.2;
    angle_res_ = 1.0 / 4.0;
    max_angle_vel_ = 0.3 * M_PI;
    enable_retreat_ = true;
    check_shot_ = false;
  }
  /* ---------- map params ---------- */
  resolution_ = 0.2;  // costmap_ptr_->getResolution();
  time_resolution_ = 0.8;
  this->inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;

  // edt_environment_->getMapRegion(origin_, map_size_3d_);
  double x_origin = costmap_ptr_->getOriginX();
  double y_origin = costmap_ptr_->getOriginY();
  origin_ << x_origin, y_origin, 0.0;

  double x_size = costmap_ptr_->getSizeInCellsX();
  double y_size = costmap_ptr_->getSizeInCellsY();
  map_size_3d_ << x_size, y_size, 0.0;

  LOG(INFO) << "origin_: " << origin_.transpose();
  LOG(INFO) << "map size: " << map_size_3d_.transpose();
  LOG(INFO) << "resolution: " << resolution_;

  dijkstar_planner_ptr_ =
      std::make_shared<DijkstraExpansion>(x_size, y_size, false);

  phi_.setZero();
  phi_.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
  use_node_num_ = 0;
  iter_num_ = 0;
  search_pose_.clear();
}

int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v,
                             Eigen::Vector3d end_pt, Eigen::Vector3d end_v,
                             bool init, bool dynamic, double) {
  start_vel_ = start_v;
  search_pose_.clear();

  int xs = costmap_ptr_->getSizeInCellsX();
  int ys = costmap_ptr_->getSizeInCellsY();
  dijkstar_planner_ptr_->outlineMap(costmap_ptr_->getCharMap(), xs, ys, 253);
  WorldmapPoint start_wm_point = {start_pt(0), start_pt(1)};
  CostmapPoint start_cp_point;
  if (!costmap_ptr_->worldToMap(start_wm_point, start_cp_point)) {
    LOG(ERROR) << "robot pose out of range";
    return false;
  }
  WorldmapPoint target_wm_point = {end_pt(0), end_pt(1)};
  CostmapPoint target_cp_point;
  if (!costmap_ptr_->worldToMap(target_wm_point, target_cp_point)) {
    LOG(ERROR) << "target pose out of range";
    return false;
  }
  //获取当前机器人位置的代价值,以此设定势场的致命值
  int cur_cost = static_cast<int>(
      costmap_ptr_->getCost(target_cp_point.ui_x, target_cp_point.ui_y));
  LOG(INFO) << "kinodynamic current position cost: " << cur_cost;
  int lethal_cost = (cur_cost > 130 && cur_cost < 190) ? cur_cost : 120;
  LOG(INFO) << "kinodynamic lethal cost: " << lethal_cost;
  if (!dijkstar_planner_ptr_->calculateAllPotentials(
          costmap_ptr_->getCharMap(), target_cp_point.ui_x,
          target_cp_point.ui_y, start_cp_point.ui_x, start_cp_point.ui_y,
          xs * ys * 2, lethal_cost)) {
    LOG(INFO) << "calcu potential failed";
    return NO_PATH;
  }

  /* ---------- initialize ---------- */
  PathNodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->state.head(3) = start_pt;
  LOG(ERROR) << "start pt: " << start_pt(0) << ", " << start_pt(1) << ", "
             << start_pt(2);
  cur_node->state.tail(3) = start_v;
  // LOG(ERROR) << "start vel: " << start_v(0) << ", " << start_v(1) << ", "
  //            << start_v(2);
  cur_node->index =
      posToIndex(Eigen::Vector3d(start_pt(0), start_pt(1), start_pt(2)));
  cur_node->g_score = 0.0;

  Eigen::VectorXd end_state(6);
  Eigen::Vector3i end_index;
  double time_to_goal = 0.0;

  end_state.head(3) = end_pt;
  LOG(ERROR) << "end pt: " << end_pt(0) << ", " << end_pt(1) << ", "
             << end_pt(2);
  end_state.tail(3) = end_v;
  // LOG(ERROR) << "end vel: " << end_v(0) << ", " << end_v(1) << ", " <<
  // end_v(2);

  end_index = posToIndex(Eigen::Vector3d(end_pt(0), end_pt(1), end_pt(2)));
  cur_node->f_score =
      lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);
  cur_node->node_state = IN_OPEN_SET;

  open_set_.push(cur_node);
  use_node_num_ += 1;

  expanded_nodes_.insert(cur_node->index, cur_node);

  // PathNodePtr neighbor = NULL;
  PathNodePtr terminate_node = NULL;
  bool init_search = init;
  const int tolerance = ceil(one_shot_dist_ / resolution_);
  // const int tolerance = 0.5;

  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {
    if (iter_num_ > 10000) {
      LOG(ERROR) << "iter_num_ > 10000"
                 << " plan failed";
      LOG(ERROR) << "************************************";
      LOG(ERROR) << "************************************";
      return REACH_HORIZON;
    }
    /* ---------- get lowest f_score node ---------- */
    cur_node = open_set_.top();

    /* ---------- determine termination ---------- */
    bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                    abs(cur_node->index(1) - end_index(1)) <= tolerance;
    if (near_end) {
      // /* one shot trajectory */
      estimateHeuristic(cur_node->state, end_state, time_to_goal);
      cur_node = computeShotTraj(cur_node, end_state, time_to_goal);
      terminate_node = cur_node;
      if (terminate_node->parent != NULL && is_shot_succ_) {
        retrievePath(cur_node);
        return REACH_END;
      }
    }

    /* ---------- pop node and add to close set ---------- */
    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;

    /* ---------- init state propagation ---------- */

    Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
    Eigen::Matrix<double, 6, 1> pro_state;
    std::vector<PathNodePtr> tmp_expand_nodes;
    Eigen::Vector2d um;
    double pro_t = 0.0;

    std::vector<Eigen::Vector2d> inputs;
    std::vector<double> durations;

    if (false) {
      um << start_v(0), start_v(2);
      inputs.push_back(um);
      for (double tau = time_res_init_ * init_max_tau_; tau <= init_max_tau_;
           tau += time_res_init_ * init_max_tau_)
        durations.push_back(tau);
    } else {
      // 如果允许向后规划，则起始速度从负值开始搜索
      double start_vel = enable_retreat_ ? -max_vel_ : 0.0;
      // double max_angle_vel = 0.6;
      for (double linear_vel = start_vel; linear_vel <= max_vel_ + 1e-3;
           linear_vel += max_vel_ * vel_res_) {
        for (double angle_vel = -max_angle_vel_;
             angle_vel <= max_angle_vel_ + 1e-3;
             angle_vel += max_angle_vel_ * angle_res_) {
          if (fabs(linear_vel) > 0.2) {
            um << linear_vel, angle_vel;
            inputs.push_back(um);
          }
        }
      }
      for (double tau = time_res_ * max_tau_; tau <= max_tau_ + 0.1;
           tau += time_res_ * max_tau_) {
        durations.push_back(tau);
      }
    }

    /* ---------- state propagation loop ---------- */
    // LOG(ERROR) << "inputs size: " << inputs.size();
    // LOG(ERROR) << "durations: " << durations.size();
    int input_size = inputs.size();
    int durations_size = durations.size();
    for (int i = 0; i < input_size; ++i)
      for (int j = 0; j < durations_size; ++j) {
        init_search = false;
        um = inputs[i];
        double tau = durations[j];
        stateTransit(cur_state, pro_state, um, tau);
        pro_t = cur_node->time + tau;

        /* ---------- check if in free space ---------- */

        /* inside map range */
        Pose2d cur_pose_2d(cur_state(0), cur_state(1), cur_state(2));
        search_pose_.push_back(cur_pose_2d);
        // LOG(INFO) << "cur_state: " << cur_state(0) << ", " << cur_state(1)
        //           << ", " << cur_state(2);
        // LOG(INFO) << "pro_state: " << pro_state(0) << ", " << pro_state(1)
        //           << ", " << pro_state(2);
        // if (pro_state(0) <= origin_(0) || pro_state(0) >= map_size_3d_(0) ||
        //     pro_state(1) <= origin_(1) || pro_state(1) >= map_size_3d_(1) ||
        //     pro_state(2) <= origin_(2) || pro_state(2) >= map_size_3d_(2)) {
        //   LOG(ERROR) << "outside map";
        //   continue;
        // }

        /* not in close set */
        Eigen::Vector3d pose(pro_state(0), pro_state(1), pro_state(2));
        Eigen::Vector3i pro_id = posToIndex(pose);
        int pro_t_id = timeToIndex(pro_t);

        PathNodePtr pro_node = expanded_nodes_.find(pro_id);

        if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET) {
          continue;
        }

        /* vel feasibe */
        // Eigen::Vector3d pro_v = pro_state.tail(3);

        /* not in the same voxel */
        Eigen::Vector3i diff = pro_id - cur_node->index;
        int diff_time = pro_t_id - cur_node->time_idx;
        if (diff.norm() == 0 && ((!dynamic) || diff_time == 0)) {
          continue;
        }

        /* collision free */
        Eigen::Vector3d pos;
        Eigen::Matrix<double, 6, 1> xt;
        bool is_occ = false;

        for (int k = 1; k <= check_num_; ++k) {
          double dt = tau * double(k) / double(check_num_);
          stateTransit(cur_state, xt, um, dt);
          pos = xt.head(3);
          pos(2) = 0;
          // double dist = 0;// edt_environment_->evaluateCoarseEDT(pos, -1.0);
          if (judgeCostValue(pos)) {
            is_occ = true;
            break;
          }
        }

        if (is_occ) {
          continue;
        }

        /* ---------- compute cost ---------- */
        double time_to_goal = 0.0, tmp_g_score = 0.0, tmp_f_score = 0.0;
        // Eigen::Vector3d pose_diff = pro_state.head(3) - cur_state.head(3);
        // tmp_g_score = pose_diff.squaredNorm() + cur_node->g_score;
        // double temp_score = fabs(pro_state(1) - cur_state(1)) +
        //                     fabs(pro_state(0) - cur_state(0)) +
        //                     2 * fabs(pro_state(2) - cur_state(2));
        if (um(0) < 0) {
          tmp_g_score = 10000 * fabs(um(0) * tau) + 0.0 * fabs(um(1) * tau);
        } else {
          tmp_g_score = fabs(um(0) * tau) + 0.1 * fabs(um(1) * tau);
        }
        // tmp_g_score = tmp_g_score + w_time_ * tau + cur_node->g_score;
        // LOG(INFO) << "tmp_g_score: " << tmp_g_score << " um: " << um(0);
        tmp_g_score = tmp_g_score + cur_node->g_score;
        tmp_f_score =
            tmp_g_score +
            lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);
        // LOG(ERROR) << "pro state: " << pro_state.transpose();
        // LOG(INFO) << "cur f score: " << tmp_f_score;
        // LOG(INFO) << "cur g score: " << tmp_g_score;
        /* ---------- compare expanded node in this loop ---------- */

        bool prune = false;
        int tmp_expande_nodes_size = tmp_expand_nodes.size();
        for (int j = 0; j < tmp_expande_nodes_size; ++j) {
          PathNodePtr expand_node = tmp_expand_nodes[j];
          if ((pro_id - expand_node->index).norm() == 0 &&
              ((!dynamic) || pro_t_id == expand_node->time_idx)) {
            prune = true;

            if (tmp_f_score < expand_node->f_score) {
              expand_node->f_score = tmp_f_score;
              expand_node->g_score = tmp_g_score;
              expand_node->state = pro_state;
              expand_node->input = um;
              expand_node->duration = tau;
            }
            break;
          }
        }

        /* ---------- new neighbor in this loop ---------- */
        if (!prune) {
          if (pro_node == NULL) {
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->state = pro_state;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->input = um;
            pro_node->duration = tau;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;

            open_set_.push(pro_node);

            expanded_nodes_.insert(pro_id, pro_node);

            tmp_expand_nodes.push_back(pro_node);
            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_) {
              LOG(ERROR) << "Run out of total number";
              return NO_PATH;
            }
          } else if (pro_node->node_state == IN_OPEN_SET) {
            if (tmp_g_score < pro_node->g_score) {
              // pro_node->index = pro_id;
              pro_node->state = pro_state;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->input = um;
              pro_node->duration = tau;
              pro_node->parent = cur_node;
            }
          } else {
            LOG(ERROR) << "error type in searching: " << pro_node->node_state;
          }
        }
        /* ----------  ---------- */
      }
  }

  /* ---------- open set empty, no path ---------- */
  LOG(ERROR) << "open set empty, no path!";
  LOG(ERROR) << "use node num: " << use_node_num_;
  LOG(ERROR) << "iter num: " << iter_num_;
  return NO_PATH;
}

void KinodynamicAstar::retrievePath(PathNodePtr end_node) {
  PathNodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL) {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}

double KinodynamicAstar::calcDubinsCost(Eigen::VectorXd x1,
                                        Eigen::VectorXd x2) {
  ompl::base::DubinsStateSpace duBinsPath(0.3);
  ompl::base::SE2StateSpace::StateType *rsStart =
      (ompl::base::SE2StateSpace::StateType *) duBinsPath.allocState();
  ompl::base::SE2StateSpace::StateType *rsEnd =
      (ompl::base::SE2StateSpace::StateType *) duBinsPath.allocState();
  rsStart->setXY(x1(0), x1(1));
  rsStart->setYaw(x1(2));
  rsEnd->setXY(x2(0), x2(1));
  rsEnd->setYaw(x2(2));
  return duBinsPath.distance(rsStart, rsEnd);
}

double KinodynamicAstar::calcReedsSheppCost(Eigen::VectorXd x1,
                                            Eigen::VectorXd x2) {
  ompl::base::ReedsSheppStateSpace reedsSheppPath(0.6);
  ompl::base::SE2StateSpace::StateType *rsStart =
      (ompl::base::SE2StateSpace::StateType *) reedsSheppPath.allocState();
  ompl::base::SE2StateSpace::StateType *rsEnd =
      (ompl::base::SE2StateSpace::StateType *) reedsSheppPath.allocState();
  rsStart->setXY(x1(0), x1(1));
  rsStart->setYaw(x1(2));
  rsEnd->setXY(x2(0), x2(1));
  rsEnd->setYaw(x2(2));
  return reedsSheppPath.distance(rsStart, rsEnd);
}

double KinodynamicAstar::estimateHeuristic(Eigen::VectorXd x1,
                                           Eigen::VectorXd x2,
                                           double &optimal_time) {
  WorldmapPoint start_wm_point = {x1(0), x1(1)};
  CostmapPoint start_cp_point;
  double djCost;
  if (costmap_ptr_->worldToMap(start_wm_point, start_cp_point)) {
    djCost = 1.0 * dijkstar_planner_ptr_->getDistancePotential(
                       start_cp_point.ui_x, start_cp_point.ui_y);
  } else {
    djCost = 1.0e9 + 0.1;
  }
  // double reedsSheppCost = calcReedsSheppCost(x1, x2);
  // double duBinCost = 1.05 * calcDubinsCost(x1, x2);

  // double cost = std::max(djCost, std::max(reedsSheppCost, duBinCost));
  // double cost = std::max(djCost, duBinCost);

  double cost;
  if (djCost < 1.0e9) {
    cost = djCost;
  } else {
    LOG(INFO) << "use dubins cost";
    cost = 10 * calcDubinsCost(x1, x2);
  }
  // LOG(INFO) << "cccccost: " << cost << "   pose: " << start_cp_point.ui_x
  //            << ", " << start_cp_point.ui_y;
  // cost += w_time_ * optimal_time;
  // LOG(INFO) << "reeds_shepp_cost: " << reedsSheppCost
  //            << "  duBinCost: " << duBinCost << "  djCost: " << djCost;
  return (1 + tie_breaker_) * cost;
}

PathNodePtr KinodynamicAstar::computeShotTraj(PathNodePtr cur_node,
                                              Eigen::VectorXd state2,
                                              double time_to_goal) {
  ompl::base::DubinsStateSpace reeds_shepp_shot(one_shot_radiu_);
  ompl::base::SE2StateSpace::StateType *rsStart =
      (ompl::base::SE2StateSpace::StateType *) reeds_shepp_shot.allocState();
  ompl::base::SE2StateSpace::StateType *rsEnd =
      (ompl::base::SE2StateSpace::StateType *) reeds_shepp_shot.allocState();
  rsStart->setXY(cur_node->state(0), cur_node->state(1));
  rsStart->setYaw(cur_node->state(2));
  rsEnd->setXY(state2(0), state2(1));
  rsEnd->setYaw(state2(2));
  double length = reeds_shepp_shot.distance(rsStart, rsEnd);
  double inc_t = one_shot_inter_dist_ / length;
  // if (length > 0.4) {
  //   inc_t = 0.3;
  // } else {
  //   inc_t = 0.3;
  // }
  if (check_shot_) {
    for (double t = 0.0; t <= 1.5; t += inc_t) {
      ompl::base::SE2StateSpace::StateType *rs_state =
          (ompl::base::SE2StateSpace::StateType *)
              reeds_shepp_shot.allocState();
      reeds_shepp_shot.interpolate(rsStart, rsEnd, t, rs_state);
      Eigen::Vector3d point(rs_state->getX(), rs_state->getY(),
                            rs_state->getYaw());
      if (judgeCostValue(point)) {
        delete rs_state;
        is_shot_succ_ = false;
        return cur_node;
      }
    }
  }
  for (double t = 0.0; t <= 1.5; t += inc_t) {
    ompl::base::SE2StateSpace::StateType *rs_state =
        (ompl::base::SE2StateSpace::StateType *) reeds_shepp_shot.allocState();
    reeds_shepp_shot.interpolate(rsStart, rsEnd, t, rs_state);
    // PathNodePtr node_ptr = new PathNode;
    PathNodePtr node_ptr = path_node_pool_[use_node_num_];
    node_ptr->duration = 0.0;
    node_ptr->input = Eigen::Vector2d(0.01, 0);
    node_ptr->state << rs_state->getX(), rs_state->getY(), rsEnd->getYaw(),
        0.0001, 0.0, 0.0;
    LOG(ERROR) << "node_ptr->state: " << node_ptr->state.transpose();
    node_ptr->parent = cur_node;
    cur_node = node_ptr;
    delete rs_state;
    use_node_num_ += 1;
    if (use_node_num_ == allocate_num_) {
      is_shot_succ_ = false;
      LOG(ERROR) << "Run out of total number";
      return cur_node;
    }
  }
  is_shot_succ_ = true;
  delete rsStart;
  delete rsEnd;
  return cur_node;
}

Eigen::Vector3i KinodynamicAstar::posToIndex(Eigen::Vector3d pt) {
  Eigen::Vector3i idx;
  idx(0) = std::floor((pt(0) - origin_(0)) * inv_resolution_);
  idx(1) = std::floor((pt(1) - origin_(1)) * inv_resolution_);
  idx(2) = std::floor((pt(2) - origin_(2)) * 10);

  return idx;
}

int KinodynamicAstar::timeToIndex(double time) {
  int idx = floor((time - time_origin_) * inv_time_resolution_);
  return idx;
}

void KinodynamicAstar::stateTransit(Eigen::Matrix<double, 6, 1> &state0,
                                    Eigen::Matrix<double, 6, 1> &state1,
                                    Eigen::Vector2d um, double tau) {
  double l_r = 0.2;
  double car_length = 0.5;
  double belta = atan(l_r * tan(um(1)) / car_length);
  double v_velocity = um(0);
  double w_velocity = v_velocity * cos(belta) * tan(um(1)) / car_length;
  Eigen::Matrix<double, 6, 1> control_matrix;
  if (fabs(tan(um(1))) > 0.01) {
    control_matrix.setZero();
    control_matrix(2) = w_velocity * tau;
    control_matrix(0) = (sin(state0(2) + control_matrix(2)) - sin(state0(2))) *
                        v_velocity / w_velocity;
    control_matrix(1) = -(cos(state0(2) + control_matrix(2)) - cos(state0(2))) *
                        v_velocity / w_velocity;
    control_matrix(3) = cos(state0(2)) * um(0);
    control_matrix(4) = sin(state0(2)) * um(0);
    control_matrix(5) = w_velocity;
    Eigen::Matrix<double, 6, 6> trans_matrix;
    trans_matrix.setZero();
    trans_matrix.block<3, 3>(0, 0).setIdentity();
    state1 = trans_matrix * state0 + control_matrix;
  } else {
    control_matrix.setZero();
    control_matrix(2) = w_velocity * tau;
    control_matrix(0) = um(0) * cos(state0(2)) * tau;
    control_matrix(1) = um(0) * sin(state0(2)) * tau;
    control_matrix(3) = cos(state0(2)) * um(0);
    control_matrix(4) = sin(state0(2)) * um(0);
    control_matrix(5) = w_velocity;
    Eigen::Matrix<double, 6, 6> trans_matrix;
    trans_matrix.setZero();
    trans_matrix.block<3, 3>(0, 0).setIdentity();
    state1 = trans_matrix * state0 + control_matrix;
  }
}

SubPath KinodynamicAstar::getKinoTraj(double delta_t) {
  SubPath path_result;
  /* ---------- get traj of searching ---------- */
  PathNodePtr node = path_nodes_.back();
  Eigen::Matrix<double, 6, 1> x0, xt;
  Eigen::Vector3d last_pose(0, 0, 0);
  while (node->parent != NULL) {
    Eigen::Vector2d ut = node->input;
    double duration = node->duration;
    x0 = node->parent->state;
    // state_list.push_back(x0.head(3));
    for (double t = duration; t >= -1e-5; t -= delta_t) {
      stateTransit(x0, xt, ut, t);
      double dist = fabs(xt(0) - last_pose(0)) + fabs(xt(1) - last_pose(1));
      if (dist > 0.05) {
        path_result.wps.push_back(Pose2d(xt(0), xt(1), xt(2)));
        path_result.wpis.push_back(WayPointInfo(ut(0), ut(1), 0.0));
      }
      last_pose = xt.head(3);
    }
    node = node->parent;
  }
  reverse(path_result.wps.begin(), path_result.wps.end());
  reverse(path_result.wpis.begin(), path_result.wpis.end());
  for (size_t index = 0; index < path_result.wps.size(); index++) {
    LOG(INFO) << "kino point: " << index << " " << path_result.wps[index].getX()
              << " " << path_result.wps[index].getY() << " "
              << path_result.wps[index].getYaw();
  }
  return path_result;
}

void KinodynamicAstar::getFirstControl(Eigen::Vector2d &control_value) {
  PathNodePtr node = path_nodes_.back();
  control_value = node->input;
}

void KinodynamicAstar::reset() {
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator>
      empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++) {
    PathNodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
}

bool KinodynamicAstar::judgeCostValue(const Eigen::Vector3d &pose) {
  WorldmapPoint wm_point = {pose(0), pose(1)};
  CostmapPoint cp_point;
  if (!costmap_ptr_->worldToMap(wm_point, cp_point)) {
    // 超出边界，认为被占用
    LOG(ERROR) << "Oversize";
    return true;
  }

  int costvalue =
      static_cast<int>(costmap_ptr_->getCost(cp_point.ui_x, cp_point.ui_y));

  if (costvalue > 120) {
    return true;
  }

  return false;
}

}  // namespace CVTE_BABOT