/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file point_sampler.cpp
 *
 *@brief cpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.3.0
 *@data 2020-09-17
 ************************************************************************/
#include "car_avoiding/point_sampler.hpp"
#include "dwa/world_model/costmap_model.hpp"

namespace CVTE_BABOT {
void PointSampler::samplePoints(const Costmap2d &costmap,
                                const Pose2d &current_pose,
                                const Pose2d &car_pose,
                                std::vector<CostmapPoint> &v_sample_points) {
  // 可视化前方的一条线段,方便判断方向,调试用
  // CostmapPoint visualized_point;
  // VisualizeDirection(costmap, current_pose, car_pose, visualized_point);

  CostmapPoint target_point;
  if (!costmap.worldToMap({car_pose.x, car_pose.y}, target_point)) {
    LOG(ERROR) << "the car's pose is out of map!";
    return;
  }

  // 1.以机器人与汽车连线的方向作为道路的方向
  double road_angle = atan2((car_pose.getY() - current_pose.getY()),
                            (car_pose.getX() - current_pose.getX()));
  Pose2d road_pose(current_pose.getX(), current_pose.getY(), road_angle);

  // 2.采样得到机器人右边的一些点
  std::vector<std::pair<CostmapPoint, CostmapPoint>> vp_line_points;
  generateLineRight(costmap, road_pose, vp_line_points);

  // 3.计算得到靠近路边的点
  computeLineEndPoint(costmap, vp_line_points, v_sample_points);
}

void PointSampler::generateLineRight(
    const Costmap2d &costmap, const Pose2d &road_pose,
    std::vector<std::pair<CostmapPoint, CostmapPoint>> &vp_line_points) {
  // 采样点的范围
  // for (double sample_x = -SAMPLE_RANGE_X; sample_x <= SAMPLE_RANGE_X;
  for (double sample_x = 0; sample_x <= SAMPLE_RANGE_X;
       sample_x += SAMPLE_STEP_X) {
    Pose2d pose_center(sample_x, 0.00, 0.00);
    auto center_pose = road_pose * pose_center;
    CostmapPoint cp_center_point;
    if (!costmap.worldToMap({center_pose.getX(), center_pose.getY()},
                            cp_center_point)) {
      LOG(ERROR) << "worldToMap  error";
      continue;
    }

    Pose2d pose_right(sample_x, -SAMPLE_RANGE_Y, 0.00);
    auto right_end_pose = road_pose * pose_right;
    CostmapPoint cp_right_right;
    if (!costmap.worldToMap({right_end_pose.getX(), right_end_pose.getY()},
                            cp_right_right)) {
      LOG(ERROR) << "worldToMap  error";
      continue;
    }

    vp_line_points.emplace_back(cp_center_point, cp_right_right);
  }
}

void PointSampler::getEndPoint(const CostmapPoint &origin_point,
                               const CostmapPoint &end_point,
                               CostmapPoint &end_point_limited) {
  CostmapPoint point_end;
  if (CostmapModel::getPtrInstance()->lineCost(
          origin_point.ui_x, origin_point.ui_y, end_point.ui_x, end_point.ui_y,
          point_end)) {
    end_point_limited.ui_x = point_end.ui_x;
    end_point_limited.ui_y = point_end.ui_y;
  } else {
    end_point_limited.ui_x = end_point.ui_x;
    end_point_limited.ui_y = end_point.ui_y;
  }
}

void PointSampler::computeLineEndPoint(
    const Costmap2d &costmap,
    const std::vector<std::pair<CostmapPoint, CostmapPoint>> &v_line_points,
    std::vector<CostmapPoint> &v_end_points) {
  auto robot_x = costmap.getSizeInCellsX() / 2,
       robot_y = costmap.getSizeInCellsY() / 2;

  for (const auto & [ cp_center, cp_right ] : v_line_points) {
    CostmapPoint cp_right_end;
    double cost = CostmapModel::getPtrInstance()->lineCost(
        robot_x, robot_y, cp_center.ui_x, cp_center.ui_y);

    // 如果机器人与起点的连线有障碍，使用机器人与终点的连线来计算，可以避免计算出穿墙的点.
    if (cost < 0 || cost > DANGEROUS_COST) {
      getEndPoint({robot_x, robot_y}, cp_right, cp_right_end);
    } else {
      getEndPoint(cp_center, cp_right, cp_right_end);
    }

    CostmapPoint end_point;
    if (computeNeareatFreePoint(costmap, cp_right_end, FREE_POINT_SEARCH_RADIUS,
                                end_point)) {
      v_end_points.push_back(end_point);
    }
  }
}

}  // namespace CVTE_BABOT
