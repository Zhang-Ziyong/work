/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file kinodynamic_astar.hpp
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

#ifndef __KINODYNAMIC_ASTAR_HPP
#define __KINODYNAMIC_ASTAR_HPP

#include <queue>
#include <vector>
#include "costmap_2d.hpp"
#include "datatype.hpp"
#include "dijkstra/D_star.hpp"
#include "pose2d/pose2d.hpp"
#include "path.hpp"

namespace CVTE_BABOT {

class KinodynamicAstar {
 public:
  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3 };

  KinodynamicAstar();
  ~KinodynamicAstar();

  /* main API */
  void setParam();
  void init(const std::string &planner);
  void reset();
  // int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
  //            Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
  //            Eigen::Vector3d end_vel, bool init, bool dynamic = false,
  //            double time_start = -1.0);
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
             Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool init,
             bool dynamic = false, double time_start = -1.0);

  SubPath getKinoTraj(double delta_t);

  void getSamples(double &ts, std::vector<Eigen::Vector3d> &point_set,
                  std::vector<Eigen::Vector3d> &start_end_derivatives);

  void getFirstControl(Eigen::Vector2d &control_value);

  std::vector<PathNodePtr> getVisitedNodes();

  inline void setCostMap(std::shared_ptr<Costmap2d> costmap) {
    costmap_ptr_ = costmap;
  }

  std::vector<Pose2d> getSearchState() { return search_pose_; };

  typedef std::shared_ptr<KinodynamicAstar> Ptr;

 private:
  std::shared_ptr<Costmap2d> costmap_ptr_ = nullptr;
  std::shared_ptr<DijkstraExpansion> dijkstar_planner_ptr_ = nullptr;

  /* ---------- main data structure ---------- */
  std::vector<PathNodePtr> path_node_pool_;
  int use_node_num_ = 0;
  int iter_num_ = 0;
  NodeHashTable expanded_nodes_;
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator>
      open_set_;
  std::vector<PathNodePtr> path_nodes_;

  /* ---------- record data ---------- */
  Eigen::Vector3d start_vel_, end_vel_, start_acc_;
  Eigen::Matrix<double, 6, 6> phi_;  // state transit matrix
  bool is_shot_succ_ = false;
  bool check_shot_ = false;
  Eigen::MatrixXd coef_shot_;
  double t_shot_ = 0.0;
  bool has_path_ = false;

  /* ---------- parameter ---------- */
  /* search */
  double max_tau_ = 0.4;
  double init_max_tau_ = 1.0;
  double max_vel_ = 1.0;
  double max_angle_vel_ = 0.3 * M_PI;
  double w_time_ = 10.0;
  double horizon_ = 7.0;
  double lambda_heu_ = 2.0;
  double margin_ = 0.2;
  int allocate_num_ = 30000;
  int check_num_ = 2;
  double tie_breaker_ = 1.0 + 1.0 / 10000;
  double one_shot_radiu_ = 0.1;
  double one_shot_dist_ = 0.6;
  double one_shot_inter_dist_ = 0.2;
  bool enable_retreat_ = true;  // 是否允许倒车
  std::vector<Pose2d> search_pose_;

  double vel_res_ = 1 / 1.0;
  double time_res_ = 1 / 1.0;
  double time_res_init_ = 1 / 8.0;
  double angle_res_ = 1 / 4.0;

  /* map */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
  double time_origin_;
  Eigen::Vector3d origin_, map_size_3d_;
  std::vector<Eigen::Vector3d> way_points;
  std::vector<int> way_points_index_;

  /* helper */
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
  int timeToIndex(double time);
  void retrievePath(PathNodePtr end_node);

  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2,
                           double &optimal_time);
  double calcDubinsCost(Eigen::VectorXd x1, Eigen::VectorXd x2);
  double calcReedsSheppCost(Eigen::VectorXd x1, Eigen::VectorXd x2);

  /* state propagation */
  void stateTransit(Eigen::Matrix<double, 6, 1> &state0,
                    Eigen::Matrix<double, 6, 1> &state1, Eigen::Vector2d um,
                    double tau);
  PathNodePtr computeShotTraj(PathNodePtr, Eigen::VectorXd state2,
                              double time_to_goal);
  bool judgeCostValue(const Eigen::Vector3d &pose);
};
}  // namespace CVTE_BABOT

#endif  // endof __KINODYNAMIC_ASTAR_HPP
