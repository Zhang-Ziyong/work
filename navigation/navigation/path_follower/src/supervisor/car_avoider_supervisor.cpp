/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file car_avoider_supervisor.cpp
 *
 *@brief cpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2020-02-21
 ************************************************************************/
#include <eigen3/Eigen/Core>

#include "costmap_mediator.hpp"
#include "perception_map.hpp"
#include "supervisor/car_avoider_supervisor.hpp"

#include "car_avoiding/point_sampler.hpp"
#include "car_avoiding/point_validity_checker.hpp"
#include "car_avoiding/visualizer.hpp"

namespace CVTE_BABOT {
CarAvoiderSupervisor::CarAvoiderSupervisor(const unsigned int &ui_size_x,
                                           const unsigned int &ui_size_y) {
  ptr_point_sampler_ = std::make_shared<PointSampler>();
  ptr_point_validity_checker_ = std::make_shared<PointValidityChecker>();
  ptr_visualizer_ = std::make_shared<Visualizer>();

  // 创建PerceptionMap
  ptr_perception_map_ = std::make_shared<PerceptionMap>(ui_size_x, ui_size_y);
  LOG(INFO) << "perception_map size_x: " << ui_size_x
            << ", size_y: " << ui_size_y;

  auto ptr_detection_result = std::make_shared<CarDetectionResult>();
  CostmapMediator::getPtrInstance()
      ->registerUpdateFunc<std::shared_ptr<CarDetectionResult>>(
          "car_detection_result", ptr_detection_result);
}

CarAvoiderSupervisor::~CarAvoiderSupervisor() {}

void CarAvoiderSupervisor::supervise(const SupervisorState &state,
                                     SupervisorResult &out) {
  if (state.state_ == PathFollowerStates::AVOID_CAR) {
    return;
  }

  const SubPath sub_path = state.sub_path_;
  if (sub_path.wps.empty()) {
    LOG(ERROR) << "CarAvoiderSupervisor : sub_path is empty.";
    return;
  }

  Pose2d current_pose = state.robot_pose_;
  // 获取车辆避让专用的costmap, 因为会以汽车的位置作为规划的终点,
  // 所以在上面清除了目标汽车的栅格, 不然无法规划.
  // 1.检查是否需要避让
  std::shared_ptr<Costmap2d> ptr_costmap_removed_car = nullptr;
  Pose2d car_pose;
  if (checkIfNeedToAvoid(state, ptr_costmap_removed_car, car_pose)) {
    // 2.采样, 得到一些可行的避让点, 核心函数
    std::vector<CostmapPoint> v_sample_points;
    ptr_point_sampler_->samplePoints(*state.ptr_costmap_2d_, current_pose,
                                     car_pose, v_sample_points);

    CostmapPoint robot_point;
    if (!ptr_costmap_removed_car->worldToMap(
            {current_pose.getX(), current_pose.getY()}, robot_point)) {
      LOG(ERROR) << "transform robot pose to map failed!!";
      return;
    }

    CostmapPoint car_point;
    if (!ptr_costmap_removed_car->worldToMap({car_pose.getX(), car_pose.getY()},
                                             car_point)) {
      LOG(ERROR) << "transform car pose to map failed!!";
      return;
    }

    // 3.评分, empty now.
    std::vector<CostmapPoint> v_valid_point;
    ptr_point_validity_checker_->checkPointsValidity(
        *ptr_costmap_removed_car, v_sample_points, car_pose, v_valid_point);

    // 3.评分, empty now.
    scoreTargetPoints();

    // 4.选取最优点
    CostmapPoint target_point;
    if (getBestTargetPoints(*state.ptr_costmap_2d_, v_valid_point, current_pose,
                            car_pose, target_point)) {
      ptr_visualizer_->visualizeSampoints(*ptr_costmap_removed_car, robot_point,
                                          car_point, v_sample_points,
                                          target_point);
      auto perception_mat = ptr_perception_map_->getVisMat();
      cv::imwrite("/home/droid/mat/perception_mat.jpg", perception_mat);

      out.can_continue = false;
      out.new_local_goal = true;
      out.status = CARAVOIDER;
      out.supervisor_name = "CarAvoider";

      WorldmapPoint wp_point;
      state.ptr_costmap_2d_->mapToWorld(target_point, wp_point);

      // 1.以机器人与汽车连线的方向作为道路的方向
      double road_angle = atan2((car_pose.getY() - current_pose.getY()),
                                (car_pose.getX() - current_pose.getX()));
      // need to rotate.
      // out.local_goal.setPose(wp_point.d_x, wp_point.d_y, road_angle);
      out.target_point.position.setPose(wp_point.d_x, wp_point.d_y, road_angle);
      LOG(INFO) << "target_goal, x = " << wp_point.d_x
                << ", y: " << wp_point.d_y;
    } else {
      LOG(ERROR) << "There is no point available to avoid the car!!";
    }
  } else {
    LOG(INFO) << "Need not to avoid car.";
  }
}

bool CarAvoiderSupervisor::checkIfNeedToAvoid(
    const SupervisorState &state, std::shared_ptr<Costmap2d> &ptr_costmap,
    Pose2d &car_pose) {
// #define GAZEBO
#ifndef GAZEBO

  auto ptr_detection_result = std::make_shared<CarDetectionResult>();
  CostmapMediator::getData("car_detection_result", ptr_detection_result);

  if (ptr_detection_result->v_camera_result.empty()) {
    LOG(INFO) << "the car detection result is empty.";
    return false;
  }

  double time = 0.00;
  if (CostmapMediator::getPtrInstance()->getCurrentTime(time)) {
    LOG(ERROR) << "getCurrentTime error!";
    return false;
  }

  double time_error = fabs(time - ptr_detection_result->d_time);
  if (time_error > 10) {
    LOG(INFO) << "The car detection result is delay, detection_result time: "
              << ptr_detection_result->d_time << ", current time: " << time
              << ", time_error: " << time_error;
    return false;
  } else {
    LOG(INFO) << "Detection_result time_error: " << time_error;
  }

  ptr_perception_map_->updateCostmap(state.ptr_costmap_2d_);
  ptr_perception_map_->updateMap();

  int index = 0;
  double dist = 0.0;
  double min_dist = DBL_MAX;

  for (size_t i = 0; i < state.sub_path_.wps.size(); ++i) {
    dist = state.robot_pose_.distanceTo(state.sub_path_.wps[i]);
    if (dist < min_dist) {
      min_dist = dist;
      index = i;
    }
  }

  for (size_t start_idx = index; start_idx < state.sub_path_.wps.size();
       ++start_idx) {
    auto path_point = state.sub_path_.wps[start_idx];
    GridStatus grid_status;
    if (ptr_perception_map_->getGridStatus(path_point.getX(), path_point.getY(),
                                           grid_status)) {
      if (grid_status.obstacle_type == Car) {
        DynamicObject object;
        if (!ptr_perception_map_->getObject(grid_status.id, object)) {
          LOG(ERROR) << "get object failed.";
          return false;
        }

        car_pose.x = object.position.d_x;
        car_pose.y = object.position.d_y;
        double distance = car_pose.distanceTo(state.robot_pose_);
        LOG(INFO) << "There is car in the path. id :" << grid_status.id
                  << ", distance: " << distance;

        // 计算速度与路径的夹角
        // if (checkIfCarApproachRobot(current_pose, path_point,
        //                             object.velocity)) {
        //   car_pose.x = path_point.getX();
        //   car_pose.y = path_point.getY();

        //   ptr_costmap =
        //       ptr_perception_map_->getCostmapResetCorGrids(grid_status.id);
        //   return true;
        // } else {
        //   LOG(INFO) << "The car is not approach the robot, needn't to
        //   avoid.";
        // }

        ptr_costmap =
            ptr_perception_map_->getCostmapResetCorGrids(grid_status.id);

        return true;
      }
    }
  }
  return false;
#else
  // 仿真用
  ptr_costmap = std::make_shared<Costmap2d>(*state.ptr_costmap_2d_);

  WorldmapPoint avoidance_point;
  if (CostmapMediator::getPtrInstance()->findDynamicCar(avoidance_point)) {
    car_pose.x = avoidance_point.d_x;
    car_pose.y = avoidance_point.d_y;
    return true;
  } else {
    return false;
  }
#endif
}

bool CarAvoiderSupervisor::checkIfCarApproachRobot(
    const Pose2d &current_pose, const Pose2d &car_pose,
    const ProcessorPoint &car_velocity) {
  Eigen::Vector2d start;
  start(0) = current_pose.getX();
  start(1) = current_pose.getY();

  Eigen::Vector2d end;
  end(0) = car_pose.getX();
  end(1) = car_pose.getY();

  Eigen::Vector2d path_direction;
  path_direction = end - start;

  Eigen::Vector2d velocity;
  velocity(0) = car_velocity.d_x;
  velocity(1) = car_velocity.d_y;
  LOG(INFO) << "car velocity x: " << car_velocity.d_x
            << ", velocity y: " << car_velocity.d_y;

  // 相乘大于0说明汽车向机器人靠近
  auto result = velocity.transpose() * path_direction;
  LOG(INFO) << "path and velocity dot product result: " << result(0);

  if (result(0) < 0) {
    return true;
  } else {
    return false;
  }
}

void CarAvoiderSupervisor::scoreTargetPoints() {}

bool CarAvoiderSupervisor::getBestTargetPoints(
    const Costmap2d &costmap, const std::vector<CostmapPoint> &v_sample_points,
    const Pose2d &current_pose, const Pose2d &car_pose,
    CostmapPoint &target_point) {
  if (v_sample_points.empty()) {
    return false;
  }

  auto robot_x = costmap.getSizeInCellsX() / 2;
  auto robot_y = costmap.getSizeInCellsY() / 2;

  // 点斜式转换成一般式： Ax + By + C = 0;
  double k = (car_pose.getY() - current_pose.getY()) /
             (car_pose.getX() - current_pose.getX());
  double A = k, B = -1, C = robot_y - k * robot_x;

  // 目前采用离得最远的点为最优路径点, 因为旋转的角度最小,
  // 到达时车头与地面的夹角也较小, 能尽量贴近路边
  double max_distance = -1e8;
  int max_point_it = -1;

  for (size_t point_it = 0; point_it < v_sample_points.size(); point_it++) {
    const double MIN_DISTANCE_TO_CAR = 3.0;

    WorldmapPoint wp_sample_point;
    costmap.mapToWorld(
        {v_sample_points[point_it].ui_x, v_sample_points[point_it].ui_y},
        wp_sample_point);

    double distance_to_car =
        car_pose.distanceTo({wp_sample_point.d_x, wp_sample_point.d_y, 0.0});
    if (distance_to_car < MIN_DISTANCE_TO_CAR) {
      LOG(INFO) << "the sample point is to close to the car, x: "
                << wp_sample_point.d_x << ", y: " << wp_sample_point.d_y
                << ", distance: " << distance_to_car << ", discard it.";
      continue;
    }

    // 点到直线距离 d=|Ax1+By1+C|/√(A²+B²)
    double x1 = v_sample_points[point_it].ui_x;
    double y1 = v_sample_points[point_it].ui_y;

    double distance = fabs(A * x1 + B * y1 + C) / sqrt(A * A + B * B);

    if (distance > max_distance) {
      max_distance = distance;
      max_point_it = point_it;
    }
  }

  // 没有找到避让点
  if (max_point_it == -1) {
    return false;
  }

  target_point = v_sample_points[max_point_it];
  return true;
}

}  // namespace CVTE_BABOT
