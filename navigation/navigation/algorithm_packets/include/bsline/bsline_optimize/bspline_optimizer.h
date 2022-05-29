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

#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>

#include <iostream>
#include <utility>
#include <random>
#include <queue>
#include <tuple>
#include <list>
#include <string>
#include <mutex>
#include <thread>

using std::cout;
using std::endl;
using std::list;
using std::max;
using std::pair;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace CVTE_BABOT {
class Costmap2d;

struct DisGradPoins {
  double pos_x;
  double pos_y;
  double distance;
  double grad_x;
  double grad_y;
};

class BsplineOptimizer {
 public:
  static const int SMOOTHNESS;
  static const int DISTANCE;
  static const int FEASIBILITY;
  static const int ENDPOINT;
  static const int GUIDE;
  static const int WAYPOINTS;

  static const int GUIDE_PHASE;
  static const int NORMAL_PHASE;

  BsplineOptimizer() {}
  ~BsplineOptimizer() {}

  double getDistance(const Eigen::Vector3d &pos);
  void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);
  void getSurroundPts(const Eigen::Vector3d &pos, Eigen::Vector3d pts[2][2][2],
                      Eigen::Vector3d &diff);
  void evaluateEDTWithGrad(const Eigen::Vector3d &pos, double time,
                           double &dist, Eigen::Vector3d &grad);
  void interpolateTrilinear(double values[2][2][2], const Eigen::Vector3d &diff,
                            double &value, Eigen::Vector3d &grad);

  void setEnvironment(const std::shared_ptr<Costmap2d> &ptr_costmap);
  /* main API */
  void setParam(double lambda1, double lambda2, double lambda3, double lambda4,
                double dist, double min_rot_radiu, int max_iter_nums,
                double max_iter_time);
  Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd &points,
                                      const double &ts,
                                      const int &cost_function, int max_num_id,
                                      int max_time_id);

  /* helper function */

  // required inputs
  void setControlPoints(const Eigen::MatrixXd &points);
  void setBsplineInterval(const double &ts);
  void setCostFunction(const int &cost_function);
  void setTerminateCond(const int &max_num_id, const int &max_time_id);

  // optional inputs
  void setGuidePath(const vector<Eigen::Vector3d> &guide_pt);
  void setWaypoints(const vector<Eigen::Vector3d> &waypts,
                    const vector<int> &waypt_idx);  // N-2 constraints at most

  void optimize();

  Eigen::MatrixXd getControlPoints();
  vector<Eigen::Vector3d> matrixToVectors(const Eigen::MatrixXd &ctrl_pts);

 private:
  std::shared_ptr<Costmap2d> ptr_costmap_;

  double resolution_ = 0.00, resolution_inv_ = 0.00;

  // main input
  Eigen::MatrixXd control_points_;  // B-spline control points, N x dim
  double bspline_interval_;         // B-spline knot span
  Eigen::Vector3d end_pt_;          // end of the trajectory
  Eigen::Vector3d end_dir_;
  int dim_;                            // dimension of the B-spline
                                       //
  vector<Eigen::Vector3d> guide_pts_;  // geometric guiding path points, N-6
  vector<Eigen::Vector3d> waypoints_;  // waypts constraints
  vector<int> waypt_idx_;              // waypts constraints index
                                       //
  int max_num_id_, max_time_id_;       // stopping criteria
  int cost_function_;                  // used to determine objective function

  /* optimization parameters */
  int order_;       // bspline degree
  double lambda1_;  // jerk smoothness weight
  double lambda2_;  // distance weight
  double lambda3_;  // feasibility weight
  double lambda4_;  // end point weight
  double lambda5_;  // guide cost weight
  double lambda6_;  // visibility cost weight
  double lambda7_;  // waypoints cost weight
  double lambda8_;  // acc smoothness
                    //
  double dist0_;    // safe distance
  // double max_vel_, max_acc_;  // dynamic limits
  double min_rot_radiu_;
  double min_cos_cost_;
  double min_rot_angle_;
  double visib_min_;              // threshold of visibility
  double wnl_;                    //
  double dlmin_;                  //
                                  //
  int algorithm1_;                // optimization algorithms for quadratic cost
  int algorithm2_;                // optimization algorithms for general cost
  int max_iteration_num_[4];      // stopping criteria that can be used
  double max_iteration_time_[4];  // stopping criteria that can be used

  /* intermediate variables */
  /* buffer for gradient of cost function, to avoid repeated allocation and
   * release of memory */
  vector<Eigen::Vector3d> g_q_;  //存储每个控制点
  vector<Eigen::Vector3d> g_smoothness_;
  vector<Eigen::Vector3d> g_distance_;
  vector<Eigen::Vector3d> g_feasibility_;
  vector<Eigen::Vector3d> g_endpoint_;
  vector<Eigen::Vector3d> g_guide_;
  vector<Eigen::Vector3d> g_waypoints_;

  int variable_num_;                   // optimization variables
  int iter_num_;                       // iteration of the solver
  std::vector<double> best_variable_;  //
  double min_cost_;                    //

  /* cost function */
  /* calculate each part of cost function with control points q as input */

  static double costFunction(const std::vector<double> &x,
                             std::vector<double> &grad, void *func_data);
  void combineCost(const std::vector<double> &x, vector<double> &grad,
                   double &cost);

  // q contains all control points
  void calcSmoothnessCost(const vector<Eigen::Vector3d> &q, double &cost,
                          vector<Eigen::Vector3d> &gradient);
  void calcDistanceCost(const vector<Eigen::Vector3d> &q, double &cost,
                        vector<Eigen::Vector3d> &gradient);
  void calcFeasibilityCost(const vector<Eigen::Vector3d> &q, double &cost,
                           vector<Eigen::Vector3d> &gradient);
  void calcEndpointCost(const vector<Eigen::Vector3d> &q, double &cost,
                        vector<Eigen::Vector3d> &gradient);
  void calcGuideCost(const vector<Eigen::Vector3d> &q, double &cost,
                     vector<Eigen::Vector3d> &gradient);
  void calcVisibilityCost(const vector<Eigen::Vector3d> &q, double &cost,
                          vector<Eigen::Vector3d> &gradient);
  void calcWaypointsCost(const vector<Eigen::Vector3d> &q, double &cost,
                         vector<Eigen::Vector3d> &gradient);
  void calcViewCost(const vector<Eigen::Vector3d> &q, double &cost,
                    vector<Eigen::Vector3d> &gradient);
  bool isQuadratic();

  /* for benckmark evaluation only */
 public:
  vector<double> vec_cost_;
  vector<double> vec_time_;
  vector<DisGradPoins> vec_dgps_;

  void getDisPoints(vector<DisGradPoins> &vec_dgps) { vec_dgps = vec_dgps_; }

  void getCostCurve(vector<double> &cost, vector<double> &time) {
    cost = vec_cost_;
    time = vec_time_;
  }

  typedef unique_ptr<BsplineOptimizer> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace CVTE_BABOT
#endif