/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file visualizer.cpp
 *
 *@brief cpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.3.0
 *@data 2020-09-17
 ************************************************************************/
#include "car_avoiding/visualizer.hpp"
#include "costmap_mediator.hpp"
#include "perception_map.hpp"

namespace CVTE_BABOT {
Visualizer::Visualizer() {
  show_debug_ = false;
  if (show_debug_) {
    thread_ = std::thread(std::bind(&Visualizer::showMat, this));
  }
}

Visualizer::~Visualizer() {
  if (show_debug_) {
    b_show_down_ = true;
    thread_.join();
  }
}

/*============================可视化用==================================*/

void Visualizer::showMatForDebug(const std::string &title, const cv::Mat &mat,
                                 const unsigned int &ms) {
  if (show_debug_) {
    cv::imshow(title, mat);
    cv::waitKey(ms);
  }
}

void Visualizer::visualizeSampoints(
    const Costmap2d &costmap, const CostmapPoint &robot_point,
    const CostmapPoint &car_point,
    const std::vector<CostmapPoint> &v_valid_point,
    const CostmapPoint &target_point) {
  cv::Mat sample_points_mat;
  getMapMat(costmap, sample_points_mat);
  drawSamplePoints(v_valid_point, sample_points_mat);
  drawRobotPoint(robot_point, sample_points_mat);
  drawCarPoint(car_point, sample_points_mat);
  drawTargetPoint(target_point, sample_points_mat);

  addMat(sample_points_mat);
  cv::imwrite("/home/droid/mat/sample_points_mat.jpg", sample_points_mat);
}

void Visualizer::showMat() {
  while (!b_show_down_) {
    std::vector<cv::Mat> v_mats;
    {
      std::lock_guard<std::mutex> lock(mutex_);

      for (const auto &mat : v_mats_) {
        v_mats.push_back(mat.clone());
      }
      v_mats_.clear();
    }

    // std::shared_ptr<Costmap2d> ptr_costmap;
    // ptr_costmap_mediator_->getData("static_costmap",
    // ptr_costmap);
    // cv::Mat costmap_static;
    // std::vector<CostmapPoint> v_points;
    // drawMatPoint(ptr_costmap, v_points, costmap_static);
    // cv::imshow("static_costmap", costmap_static);
    // cv::waitKey(32);

    // 画机器人与汽车
    // Mat costmap_mat_line =
    //     costmap_mat.clone();  // 直接用=赋值是浅拷贝,共用内存空间
    // for (size_t j = 0; j < costmap_mat.cols; j++) {
    //   for (size_t i = 0; i < costmap_mat.rows; i++) {
    //     if (costmap_mat.at<unsigned char>(j, i) == LETHAL_OBSTACLE) {
    //       circle(costmap_mat_line, Point(j, i), 1, Scalar(255, 0, 0), 2);
    //     } else if (costmap_mat.at<unsigned char>(j, i) != FREE_SPACE) {
    //       circle(costmap_mat_line, Point(j, i), 1, Scalar(128, 0, 0), 1);
    //     }
    //   }
    // }
    // for (size_t i = 0; i < v_lines.size(); i++) {
    //   circle(costmap_mat_line, Point(v_lines[i].x1, v_lines[i].y1), 1,
    //          Scalar(0, 255, 0), 4);
    //   circle(costmap_mat_line, Point(v_lines[i].x0, v_lines[i].y0), 1,
    //          Scalar(0, 255, 255), 4);
    // }
    // showMatForDebug("costmap_mat_line", costmap_mat_line, 2);

    for (size_t i = 0; i < v_mats.size(); i++) {
      showMatForDebug(std::string("sample_point_mat") + std::to_string(i),
                      v_mats[i], 50);
    }
    usleep(200000);
  }
}

void Visualizer::drawSamplePoints(
    const std::vector<CostmapPoint> &v_draw_points,
    cv::Mat &costmap_mat_point) {
  auto size_x = costmap_mat_point.cols;
  auto size_y = costmap_mat_point.rows;

  for (size_t i = 0; i < v_draw_points.size(); i++) {
    // if (v_draw_points.size() == 1) {
    //   circle(costmap_mat_point,
    //          cv::Point(v_draw_points[i].ui_x, v_draw_points[i].ui_y), 3,
    //          cv::Scalar(255, 0, 255), 1);
    // } else {
    circle(costmap_mat_point, cv::Point(size_y - v_draw_points[i].ui_y,
                                        size_x - v_draw_points[i].ui_x),
           1, cv::Scalar(0, 0, 255), 1);
  }
}

void Visualizer::drawRobotPoint(const CostmapPoint &robot_point,
                                cv::Mat &costmap_mat_point) {
  auto size_x = costmap_mat_point.cols;
  auto size_y = costmap_mat_point.rows;

  circle(costmap_mat_point,
         cv::Point(size_y - robot_point.ui_y, size_x - robot_point.ui_x), 3,
         cv::Scalar(255, 0, 255), 1);
}

void Visualizer::drawCarPoint(const CostmapPoint &car_point,
                              cv::Mat &costmap_mat_point) {
  auto size_x = costmap_mat_point.cols;
  auto size_y = costmap_mat_point.rows;

  circle(costmap_mat_point,
         cv::Point(size_y - car_point.ui_y, size_x - car_point.ui_x), 1,
         cv::Scalar(0, 255, 0), 1);
}

void Visualizer::drawTargetPoint(const CostmapPoint &target_point,
                                 cv::Mat &costmap_mat_point) {
  auto size_x = costmap_mat_point.cols;
  auto size_y = costmap_mat_point.rows;

  circle(costmap_mat_point,
         cv::Point(size_y - target_point.ui_y, size_x - target_point.ui_x), 1,
         cv::Scalar(255, 255, 0), 2);
}

void Visualizer::addMat(const cv::Mat &mat) {
  std::lock_guard<std::mutex> lock(mutex_);
  v_mats_.push_back(mat.clone());
}

void Visualizer::getMapMat(const Costmap2d &costmap, cv::Mat &mat) {
  unsigned int size_x = costmap.getSizeInCellsX();
  unsigned int size_y = costmap.getSizeInCellsY();
  mat = cv::Mat::zeros(size_x, size_y, CV_8UC3);
  for (size_t j = size_y - 1; j > 0; j--) {
    for (size_t i = size_x - 1; i > 0; i--) {
      if (costmap.getCost(i, j) == LETHAL_OBSTACLE) {
        circle(mat, cv::Point(size_y - j, size_x - i), 1, cv::Scalar(255, 0, 0),
               2);
      } else if (costmap.getCost(i, j) != FREE_SPACE) {
        circle(mat, cv::Point(size_y - j, size_x - i), 1, cv::Scalar(128, 0, 0),
               2);
      }
    }
  }
}

// void Visualizer::VisualizeDirection(const std::shared_ptr<Costmap2d>
// &costmap,
//                                     const Pose2d &current_pose,
//                                     const Pose2d &target_pose,
//                                     CostmapPoint &target_point) {
//   v_lines_.clear();
//   CostmapPoint cp_current_pose;
//   if (!(costmap.worldToMap({current_pose.x, current_pose.y},
//                             cp_current_pose))) {
//     LOG(ERROR) << "worldToMap  error";
//   }

//   double cos_yaw = cos(current_pose.getYaw());
//   double sin_yaw = sin(current_pose.getYaw());

//   WorldmapPoint wp_direction;
//   // 可视化机器人坐标系下坐标为(8,0)的点
//   wp_direction.d_x = cos_yaw * (8) - sin_yaw * 0 + current_pose.x;
//   wp_direction.d_y = sin_yaw * (8) + cos_yaw * 0 + current_pose.y;

//   CostmapPoint cp_direction;
//   if (!(costmap->worldToMap({target_pose.getX(), target_pose.getY()},
//                             cp_direction))) {
//     LOG(ERROR) << "worldToMap  error";
//   }
//   target_point.ui_x = cp_direction.ui_x;
//   target_point.ui_y = cp_direction.ui_y;

//   unsigned int size_x = costmap->getSizeInCellsX();
//   unsigned int size_y = costmap->getSizeInCellsY();
//   Line line;
//   line.x0 = size_y - cp_current_pose.ui_y;
//   line.y0 = size_x - cp_current_pose.ui_x;
//   line.x1 = size_y - cp_direction.ui_y;
//   line.y1 = size_x - cp_direction.ui_x;

//   v_lines_.push_back(line);
// }

}  // namespace CVTE_BABOT
