/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file pit_planner.hpp
 *
 *@brief 过坎规划算法
 *
 *@modified by
 *
 *@author lizhongjia(lizhongjia@cvte.com)
 *@version Navigation-v2.0
 *@data 2022-04-18
 ************************************************************************/

#include "pit_planner/pit_planner.hpp"

namespace CVTE_BABOT {

bool PitPlanner::GetAllAccessPath(std::vector<Eigen::Vector2d> &start_points,
                                  std::vector<Eigen::Vector2d> &end_points) {
  if (access_interval_ == 0) {
    LOG(INFO) << "access_interval is 0";
    return false;
  }
  LOG(INFO) << "access_interval_ = " << access_interval_;
  buildConversionMatrix();
  Eigen::Matrix<double, 2, 2> input;
  input.col(0) = pit_start_;
  input.col(1) = pit_end_;
  for (int i = 0; i < input.cols(); i++) {
    input.col(i) = T_wb_.block<2, 2>(0, 0).reverse() *
                   (input.col(i) - T_wb_.block<2, 1>(0, 2));
  }

  int access_total = length_ / access_interval_;
  double edge = edge_dist_ + width_ / 2;

  int i = 0;
  int dir = input.col(0).y() < 0 ? 1 : -1;
  double l_x = input.col(0).x();
  double r_x = input.col(0).x() + access_interval_;

  Eigen::Matrix2Xd begin;
  Eigen::Matrix2Xd end;
  begin.resize(2, access_total);
  end.resize(2, access_total);
  double halt_length = length_ / 2;
  while (i < access_total) {
    if (l_x > -halt_length) {
      begin.col(i) << l_x, -edge * dir;
      end.col(i++) << l_x, edge * dir;
      l_x -= access_interval_;
    }
    if (i < access_total && r_x < halt_length) {
      begin.col(i) << r_x, -edge * dir;
      end.col(i++) << r_x, edge * dir;
      r_x += access_interval_;
    }
  }

  begin = T_wb_.block<2, 2>(0, 0) * begin;
  begin.row(0) = begin.row(0).array() + T_wb_.block<2, 1>(0, 2).x();
  begin.row(1) = begin.row(1).array() + T_wb_.block<2, 1>(0, 2).y();
  end = T_wb_.block<2, 2>(0, 0) * end;
  end.row(0) = end.row(0).array() + T_wb_.block<2, 1>(0, 2).x();
  end.row(1) = end.row(1).array() + T_wb_.block<2, 1>(0, 2).y();

  start_points.clear();
  end_points.clear();
  for (int i = 0; i < begin.cols(); i++) {
    start_points.push_back(begin.col(i));
    end_points.push_back(end.col(i));
  }
  return true;
}

bool PitPlanner::GetOptimalAccessPoint(Eigen::Vector2d &start_point,
                                       Eigen::Vector2d &end_point) {
  if (!GetAllAccessPath(start_points_, end_points_)) {
    return false;
  }
  std::vector<PitScore> scores;
  if (!pathScore(230, 20, scores)) {
    return false;
  }
  int index = 0;
  optimalPathIndex(scores, index);
  start_point = start_points_[index];
  end_point = end_points_[index];
  return true;
}

void PitPlanner::buildConversionMatrix(void) {
  double hcos = cos(heading_);
  double hsin = sin(heading_);
  double xOffset = box_center_.x();
  double yOffset = box_center_.y();
  T_wb_ << hcos, -hsin, xOffset, hsin, hcos, yOffset;
}

bool PitPlanner::pathScore(const double dangerous_cost, size_t path_num,
                           std::vector<PitScore> &scores) {
  scores.clear();
  double cost_total = 0;
  path_num = (path_num == 0) ? 1 : path_num;
  double sort_score = 0.5 / path_num;
  for (size_t j = 0; j < start_points_.size(); j++) {
    // std::cout << "index = " << j << ",pit_start " << start_points_[j].x()
    //           << "," << start_points_[j].y() << ", pit_end "
    //           << end_points_[j].x() << "," << end_points_[j].y() <<
    //           std::endl;
    int check_total = (end_points_[j] - start_points_[j]).norm() / 0.1;
    Eigen::Vector2d pose_inc;
    pose_inc = (end_points_[j] - start_points_[j]) / check_total;
    cost_total = 0;

    int i;
    for (i = 0; i < check_total; i++) {
      double cost = ptr_costmap_->getCostWithMapPose(
          start_points_[j].x() + i * pose_inc.x(),
          start_points_[j].y() + i * pose_inc.y());
      if (cost > dangerous_cost) {
        break;
      }
      cost_total += cost;
    }
    if (i == check_total) {
      PitScore score;
      score.grade = 1 - (cost_total / (255 * i)) - sort_score * scores.size();
      score.index = j;
      scores.emplace_back(score);
      if (scores.size() > path_num) {
        break;
      }
    }
  }
  return scores.size() == 0 ? false : true;
}

void PitPlanner::optimalPathIndex(std::vector<PitScore> &scores, int &index) {
  double grade_max = std::numeric_limits<double>::min();
  for (auto score : scores) {
    if (grade_max < score.grade) {
      grade_max = score.grade;
      index = score.index;
    }
  }
}

void PitPlanner::SetWayPoint(const Pose2d &pit_start, const Pose2d &pit_end) {
  pit_start_ << pit_start.getX(), pit_start.getY();
  pit_end_ << pit_end.getX(), pit_end.getY();
}

void PitPlanner::SetBox(Eigen::Vector2d box_center, double length, double width,
                        double heading) {
  box_center_ = box_center;
  length_ = length;
  width_ = width;
  heading_ = heading;
}

void PitPlanner::SetCostMap(const std::shared_ptr<Costmap2d> &costmap) {
  ptr_costmap_ = costmap;
}

}  // namespace CVTE_BABOT
