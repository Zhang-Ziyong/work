/************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2018, CVTE.
* All rights reserved.
*
*@file simple_scored_sampling_planner.cpp
*
*@author TKruse

*@modified caoyong (caoyong@cvte.com)
*@current_algo.dev.1.0
*@data 2019-04-12
************************************************************************/
#include "dwa/trajectory/simple_trajectory_scorer.hpp"
#include <glog/logging.h>
#include "dwa/cost_function/cost_function.hpp"
#include "dwa/trajectory/simple_trajectory_generator.hpp"
#include "trajectory.hpp"

namespace CVTE_BABOT {

SimpleTrajectoryScorer::SimpleTrajectoryScorer(
    const std::shared_ptr<SimpleTrajectoryGenerator> gen_list,
    const std::vector<std::shared_ptr<CostFunction>> &critics) {
  gen_list_ = gen_list;
  critics_ = critics;
}

double SimpleTrajectoryScorer::scoreTrajectory(const Trajectory &traj,
                                               const double &best_traj_cost) {
  double traj_cost = 0;
  int gen_id = 0;
  double cost = 0.0;
  for (unsigned int i = 0; i < critics_.size(); i++) {
    std::shared_ptr<CostFunction> score_function_p(critics_.at(i));
    if (score_function_p == nullptr) {
      LOG(INFO) << "score_function_p == nullptr at " << i;
      continue;
    }
    if (score_function_p->getScale() == 0) {
      continue;
    }
    cost = score_function_p->scoreTrajectory(traj);
    // LOG(INFO) << "score_function_p: " << score_function_p->getName() << "
    // cost: " << cost;
    if (cost < 0) {
      // LOG(WARNING) << "Velocity discarded by cost function" << std::endl;
      traj_cost = cost;
      break;
    }
    if (cost != 0) {
      cost *= score_function_p->getScale();
    }
    traj_cost += cost;
    // LOG(INFO) << "traj_cost : " << traj_cost;
    if (best_traj_cost > 0) {
      // since we keep adding positives, once we are worse than the best, we
      // will stay worse
      if (traj_cost > best_traj_cost) {
        break;
      }
    }
    gen_id++;
  }
  return traj_cost;
}

bool SimpleTrajectoryScorer::findBestTrajectory(
    Trajectory &traj, std::shared_ptr<std::vector<Trajectory>> all_explored) {
  Trajectory loop_traj;
  Trajectory best_traj;
  double loop_traj_cost = 0.0;
  double best_traj_cost = -1.0;
  bool gen_success = false;

  for (unsigned int i = 0; i < critics_.size(); i++) {
    if ((critics_.at(i) != nullptr) && (critics_.at(i)->prepare() == false)) {
      LOG(WARNING) << "A scoring function failed to prepare" << std::endl;
      return false;
    } 
  }

  while (gen_list_->hasMoreTrajectories()) {
    gen_success = gen_list_->nextTrajectory(loop_traj);
    if (gen_success == false) {
      // TODO(XXX): use this for debugging
      continue;
    }

    // 去除线速度为0，旋转速度过小的路径，因为机器人容易转不动
    if (loop_traj.getPointsSize() > 0) {
      double px = 0.0, py = 0.0, pth = 0.0, vx = 0.0, vy = 0.0, vth = 0.0;
      loop_traj.getPoint(0, px, py, pth, vx, vy, vth);
      if ((-0.005 < vx && vx < 0.005) && (-0.1 < vth && vth < 0.1)) {
        // LOG(INFO) << "Needn't zero path. (" << vx << ", " << vth << ")";
        continue;
      }
    }

    loop_traj_cost = scoreTrajectory(loop_traj, best_traj_cost);
    // LOG_EVERY_N(INFO, 20) << "Score:  " << loop_traj_cost;
    // double px = 0.0, py = 0.0, pth = 0.0, vx = 0.0, vy = 0.0, vth = 0.0;
    // for (unsigned int i = 0; i < loop_traj.getPointsSize(); i++) {
    // loop_traj.getPoint(0, px, py, pth, vx, vy, vth);
    // LOG_EVERY_N(INFO, 20) << vx << ", " << vy << ", " << vth;
    // }

    if (all_explored != nullptr) {
      loop_traj.cost_ = loop_traj_cost;
      all_explored->push_back(loop_traj);
    }
    if (loop_traj_cost >= 0) {
      if (best_traj_cost < 0 || loop_traj_cost < best_traj_cost) {
        best_traj_cost = loop_traj_cost;
        best_traj = loop_traj;
      }
    }
  }

  if (best_traj_cost >= 0) {
    LOG(INFO) << "Best traject cost : " << best_traj_cost << std::endl;
    traj.xv_ = best_traj.xv_;
    traj.yv_ = best_traj.yv_;
    traj.thetav_ = best_traj.thetav_;
    traj.cost_ = best_traj_cost;
    traj.resetPoints();
    double px = 0.0, py = 0.0, pth = 0.0, vx = 0.0, vy = 0.0, vth = 0.0;
    for (unsigned int i = 0; i < best_traj.getPointsSize(); i++) {
      best_traj.getPoint(i, px, py, pth, vx, vy, vth);
      // LOG(INFO) << px << ", " << py << ", " << pth << ", " << vx << ", " <<
      // vy << ", " << vth;
      traj.addPoint(px, py, pth, vx, vy, vth);
    }
  } else {
    LOG(INFO) << "Failed to find a good traj, cost : " << best_traj_cost
              << std::endl;
  }
  return best_traj_cost >= 0;
}

}  // namespace CVTE_BABOT
