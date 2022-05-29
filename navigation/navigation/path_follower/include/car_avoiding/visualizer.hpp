/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file visualizer.hpp
 *
 *@brief hpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.3.0
 *@data 2020-09-17
 ************************************************************************/
#ifndef __VISUALIZER_HPP
#define __VISUALIZER_HPP
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>

#include "car_avoiding/car_avoiding_util.hpp"
namespace CVTE_BABOT {

struct Line {
  int x0 = 0;
  int y0 = 0;
  int x1 = 0;
  int y1 = 0;
};

class Visualizer {
 public:
  Visualizer();
  ~Visualizer();

  Visualizer(const Visualizer &) = delete;
  Visualizer &operator=(const Visualizer &) = delete;

  /*============================可视化用==================================*/
  void showMatForDebug(const std::string &title, const cv::Mat &mat,
                       const unsigned int &ms);

  void showMat();

  void visualizeSampoints(const Costmap2d &costmap,
                          const CostmapPoint &robot_point,
                          const CostmapPoint &car_point,
                          const std::vector<CostmapPoint> &v_valid_point,
                          const CostmapPoint &target_point);

  void addMat(const cv::Mat &mat);

 private:
  void getMapMat(const Costmap2d &costmap, cv::Mat &mat);

  // void VisualizeDirection(const std::shared_ptr<Costmap2d> &costmap,
  //                         const Pose2d &current_pose, const Pose2d
  //                         &target_pose,
  //                         CostmapPoint &target_point);

  // bool transformPathToMap(const std::shared_ptr<Costmap2d> &costmap,
  //                         const std::vector<Pose2d> &v_path,
  //                         std::vector<CostmapPoint> &v_cp_path);

  void drawSamplePoints(const std::vector<CostmapPoint> &v_draw_points,
                        cv::Mat &costmap_mat_point);

  void drawRobotPoint(const CostmapPoint &robot_point,
                      cv::Mat &costmap_mat_point);

  void drawCarPoint(const CostmapPoint &car_point, cv::Mat &costmap_mat_point);

  void drawTargetPoint(const CostmapPoint &target_point,
                       cv::Mat &costmap_mat_point);
  bool show_debug_ = false;
  bool b_show_down_ = false;

  std::thread thread_;
  std::mutex mutex_;

  std::vector<cv::Mat> v_mats_;
};
}  // namespace CVTE_BABOT
#endif
