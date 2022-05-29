/*
* brief : map_grid.cpp 文件的单元测试
*
* create: 2019-04-15
*
* tester: caoyong
*/

#ifndef TEST_MAP_GRID_HPP
#define TEST_MAP_GRID_HPP

#include <gtest/gtest.h>

#include "map_cell.hpp"
#include "map_grid.hpp"

#include "costmap2d_builder.hpp"
#include "costmap2d_director.hpp"
#include "costmap_mediator.hpp"
#include "costmap_testing_helper.hpp"
#include "world_map_data.hpp"

#include <iostream>
#include <pose2d/pose2d.hpp>

namespace CVTE_BABOT {

/**
 * @brief 测试说明
 * １．栅格地图的初始化
 * */

TEST(MapGridTest, initNull) {
  MapGrid map_grid;
  EXPECT_EQ(0, map_grid.getSizeX());
  EXPECT_EQ(0, map_grid.getSizeY());
}

/**
 * @brief 测试说明
 * １．栅格地图的赋值
 * */

TEST(MapGridTest, operatorBrackets) {
  MapGrid map_grid(10, 10);
  map_grid(3, 5).target_dist_ = 5;
  EXPECT_EQ(5, map_grid.getCell(3, 5).target_dist_);
}

/**
 * @brief 测试说明
 * １．栅格地图的拷贝构造
 * */

TEST(MapGridTest, copyConstructor) {
  MapGrid map_grid(10, 10);
  map_grid(3, 5).target_dist_ = 5;
  MapGrid map_grid_two;
  map_grid_two = map_grid;
  EXPECT_EQ(5, map_grid_two.getCell(3, 5).target_dist_);
}

/**
 * @brief 测试说明
 * １．栅格地图的下标索引
 * */

TEST(MapGridTest, getIndex) {
  MapGrid map_grid(10, 10);
  EXPECT_EQ(53, map_grid.getIndex(3, 5));
}

/**
 * @brief 测试说明
 * １．栅格地图的复位
 * */

TEST(MapGridTest, reset) {
  MapGrid map_grid(10, 10);
  map_grid(0, 0).target_dist_ = 1;
  map_grid(0, 0).target_mark_ = true;
  map_grid(0, 0).within_robot_ = true;
  map_grid(3, 5).target_dist_ = 1;
  map_grid(3, 5).target_mark_ = true;
  map_grid(3, 5).within_robot_ = true;
  map_grid(9, 9).target_dist_ = 1;
  map_grid(9, 9).target_mark_ = true;
  map_grid(9, 9).within_robot_ = true;
  EXPECT_EQ(1, map_grid(0, 0).target_dist_);
  EXPECT_EQ(true, map_grid(0, 0).target_mark_);
  EXPECT_EQ(true, map_grid(0, 0).within_robot_);
  EXPECT_EQ(1, map_grid(3, 5).target_dist_);
  EXPECT_EQ(true, map_grid(3, 5).target_mark_);
  EXPECT_EQ(true, map_grid(3, 5).within_robot_);
  EXPECT_EQ(1, map_grid(9, 9).target_dist_);
  EXPECT_EQ(true, map_grid(9, 9).target_mark_);
  EXPECT_EQ(true, map_grid(9, 9).within_robot_);

  map_grid.resetPathDist();

  EXPECT_EQ(map_grid.unreachableCellCosts(), map_grid(9, 9).target_dist_);
  EXPECT_EQ(false, map_grid(9, 9).target_mark_);
  EXPECT_EQ(false, map_grid(9, 9).within_robot_);
  EXPECT_EQ(map_grid.unreachableCellCosts(), map_grid(3, 5).target_dist_);
  EXPECT_EQ(false, map_grid(3, 5).target_mark_);
  EXPECT_EQ(false, map_grid(3, 5).within_robot_);
  EXPECT_EQ(map_grid.unreachableCellCosts(), map_grid(0, 0).target_dist_);
  EXPECT_EQ(false, map_grid(0, 0).target_mark_);
  EXPECT_EQ(false, map_grid(0, 0).within_robot_);
}

/**
 * @brief 测试说明
 * １．栅格地图的遍历构造
 * */

TEST(MapGridTest, properGridConstruction) {
  MapGrid mg(10, 10);
  MapCell mc;

  for (int i = 0; i < 10; ++i) {
    for (int j = 0; j < 10; ++j) {
      EXPECT_FLOAT_EQ(mg(i, j).cx_, i);
      EXPECT_FLOAT_EQ(mg(i, j).cy_, j);
    }
  }
}

/**
 * @brief 测试说明
 * １．栅格地图的边界检验
 * */

TEST(MapGridTest, sizeCheck) {
  MapGrid mg(10, 10);
  MapCell mc;

  mg.sizeCheck(20, 25);

  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 25; ++j) {
      EXPECT_FLOAT_EQ(mg(i, j).cx_, i);
      EXPECT_FLOAT_EQ(mg(i, j).cy_, j);
    }
  }
}

/**
 * @brief 测试说明
 * １．栅格地图的空路径调整
 * */

TEST(MapGridTest, adjustPlanEmpty) {
  MapGrid mg(10, 10);
  std::vector<Pose2d> global_plan_in;
  std::vector<Pose2d> global_plan_out;
  double resolution = 0;
  mg.adjustPlanResolution(global_plan_in, global_plan_out, resolution);
  EXPECT_EQ(0, global_plan_out.size());
}

/**
 * @brief 测试说明
 * １．栅格地图的路径调整
 * */

TEST(MapGridTest, adjustPlan) {
  MapGrid mg(10, 10);
  std::vector<Pose2d> global_plan_in;
  std::vector<Pose2d> global_plan_out;
  double resolution = 1;
  Pose2d start;
  start.setX(1);
  start.setY(1);
  Pose2d end;
  end.setX(5);
  end.setY(5);
  global_plan_in.push_back(start);
  global_plan_in.push_back(end);
  mg.adjustPlanResolution(global_plan_in, global_plan_out, resolution);

  EXPECT_EQ(1, global_plan_out[0].getX());
  EXPECT_EQ(1, global_plan_out[0].getY());
  EXPECT_EQ(5, global_plan_out.back().getX());
  EXPECT_EQ(5, global_plan_out.back().getY());

  for (unsigned int i = 1; i < global_plan_out.size(); ++i) {
    Pose2d &p0 = global_plan_out[i - 1];
    Pose2d &p1 = global_plan_out[i];
    double d = hypot(p0.getX() - p1.getX(), p0.getY() - p1.getY());
    EXPECT_GT(d, resolution);
  }
}

/**
 * @brief 测试说明
 * １．栅格地图的路径调整，针对不同的路径分辨率
 * */

TEST(MapGridTest, adjustPlan2) {
  std::vector<Pose2d> base_plan, result;
  Pose2d pose0;
  Pose2d pose1;
  // Push two points, at (0,0) and (0,1). Gap is 1 meter
  base_plan.push_back(pose0);
  base_plan.push_back(pose1);
  base_plan.back().setY(1.0);

  // resolution >= 1, path won't change
  MapGrid::adjustPlanResolution(base_plan, result, 2.0);
  EXPECT_EQ(2, result.size());
  result.clear();
  MapGrid::adjustPlanResolution(base_plan, result, 1.0);
  EXPECT_EQ(2, result.size());
  result.clear();

  // 0.5 <= resolution < 1.0, one point should be added in the middle
  MapGrid::adjustPlanResolution(base_plan, result, 0.8);
  EXPECT_EQ(2, result.size());
  result.clear();
  MapGrid::adjustPlanResolution(base_plan, result, 0.5);
  EXPECT_EQ(2, result.size());
  result.clear();

  // 0.333 <= resolution < 0.5, two points should be added in the middle
  MapGrid::adjustPlanResolution(base_plan, result, 0.34);
  EXPECT_EQ(2, result.size());
  result.clear();

  // 0.25 <= resolution < 0.333, three points should be added in the middle
  MapGrid::adjustPlanResolution(base_plan, result, 0.32);
  EXPECT_EQ(2, result.size());
  result.clear();

  MapGrid::adjustPlanResolution(base_plan, result, 0.1);
  EXPECT_EQ(8, result.size());
  result.clear();
}

/**
 * @brief 测试说明
 * １．栅格地图的距离计算
 * */

TEST(MapGridTest, distancePropagation) {
  std::shared_ptr<LayeredCostmap> ptr_layers_ = nullptr;
  std::shared_ptr<CostmapMediator> ptr_costmap_mediator_;
  std::shared_ptr<CostmapParameters> ptr_costmap_params_;
  LayerParammeter layer_param_;
  std::shared_ptr<LayeredCostmap> ptr_layered_costmap_;
  std::shared_ptr<Costmap2d> ptr_costmap_;
  WorldmapPose robot_current_pose_;

  ptr_costmap_mediator_ = CostmapMediator::getPtrInstance();
  std::shared_ptr<WorldmapData> ptr_wm_data;
  ptr_costmap_mediator_->registerUpdateFunc<std::shared_ptr<WorldmapData>>(
      "static_map", ptr_wm_data);
  std::string layer_name("static_layer");
  std::string inf_layer_name("inflation_layer");
  bool track_unknown_space = true;
  bool use_maximum = false;
  bool trinary_costmap = true;
  int lethal_cost_threshold = 100;
  int unknown_cost_value = 255;

  ptr_costmap_params_ = std::make_shared<CostmapParameters>();
  ptr_costmap_params_->setParam(layer_name + ".enabled", true);
  ptr_costmap_params_->setParam(layer_name + ".track_unknown_space",
                                track_unknown_space);
  ptr_costmap_params_->setParam(layer_name + ".use_maximum", use_maximum);
  ptr_costmap_params_->setParam(layer_name + ".lethal_cost_threshold",
                                lethal_cost_threshold);
  ptr_costmap_params_->setParam(layer_name + ".unknown_cost_value",
                                unknown_cost_value);
  ptr_costmap_params_->setParam(layer_name + ".trinary_costmap",
                                trinary_costmap);

  layer_param_.addLayer(CVTE_BABOT::StaticLayerType, layer_name);

  auto ptr_mapdata_ = std::make_shared<WorldmapData>();
  ptr_mapdata_->ui_width_ = 10;
  ptr_mapdata_->ui_height_ = 10;
  ptr_mapdata_->d_resolution_ = 1.0;
  ptr_mapdata_->ptr_data_->resize(ptr_mapdata_->ui_width_ *
                                  ptr_mapdata_->ui_height_);
  ptr_costmap_mediator_->updateData("static_map", ptr_mapdata_);
  ptr_costmap_mediator_->setCostmapParameters(ptr_costmap_params_);
  ptr_layered_costmap_ = std::make_shared<LayeredCostmap>(false, false);

  double map_width_meters = 10.0, map_height_meters = 10.0, origin_x = 0.0,
         origin_y = 0.0;
  if (!ptr_layered_costmap_->isSizeLocked()) {
    ptr_layered_costmap_->resizeMap((unsigned int)(map_width_meters / 1.0),
                                    (unsigned int)(map_height_meters / 1.0),
                                    1.0, origin_x, origin_y);
  }

  ptr_costmap_mediator_->setLayeredCostmap(ptr_layered_costmap_);
  ptr_costmap_ =
      std::make_shared<Costmap2d>(*(ptr_costmap_mediator_->getCostmap()));

  auto builder = std::make_shared<Costmap2dBuilder>(ptr_layered_costmap_);
  auto director = std::make_shared<Costmap2dDirector>(builder);
  director->buildCostmap(layer_param_);

  robot_current_pose_.d_x = 0.0;
  robot_current_pose_.d_y = 0.0;
  robot_current_pose_.d_yaw = 0.0;
  ptr_layered_costmap_->updateMap(robot_current_pose_);

  ptr_costmap_ =
      std::make_shared<Costmap2d>(*(ptr_costmap_mediator_->getCostmap()));

  MapGrid mg(10, 10);

  // printMap(*(ptr_costmap));
  std::queue<MapCell *> dist_queue;
  mg.computeTargetDistance(dist_queue, ptr_costmap_);
  EXPECT_EQ(false, mg(0, 0).target_mark_);

  MapCell &mc = mg.getCell(0, 0);
  mc.target_dist_ = 0.0;
  mc.target_mark_ = true;
  dist_queue.push(&mc);
  mg.computeTargetDistance(dist_queue, ptr_costmap_);
  EXPECT_EQ(true, mg(0, 0).target_mark_);
  EXPECT_EQ(0.0, mg(0, 0).target_dist_);
  EXPECT_EQ(true, mg(1, 1).target_mark_);
  EXPECT_EQ(2.0, mg(1, 1).target_dist_);
  EXPECT_EQ(true, mg(0, 4).target_mark_);
  EXPECT_EQ(4.0, mg(0, 4).target_dist_);
  EXPECT_EQ(true, mg(4, 0).target_mark_);
  EXPECT_EQ(4.0, mg(4, 0).target_dist_);
  EXPECT_EQ(true, mg(9, 9).target_mark_);
  EXPECT_EQ(18.0, mg(9, 9).target_dist_);
}

}  // namespace CVTE_BABOT

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

#endif