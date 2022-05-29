/*
* brief : obstacle_cost_function.cpp 文件的单元测试
* create: 2019-04-16
* tester: caoyong
*/

#ifndef TEST_OBSTACLE_COST_FUNCTION_HPP
#define TEST_OBSTACLE_COST_FUNCTION_HPP

#include <gtest/gtest.h>
#include <memory>
#include "costmap2d_builder.hpp"
#include "costmap2d_director.hpp"
#include "costmap_mediator.hpp"
#include "costmap_model.hpp"
#include "costmap_model.hpp"
#include "costmap_testing_helper.hpp"
#include "obstacle_cost_function.hpp"
#include "trajectory.hpp"
#include "world_map_data.hpp"

namespace CVTE_BABOT {

class ObstacleCostFunctionTest : public testing::Test {
 public:
  virtual void TestBody() {}

  ObstacleCostFunctionTest() {
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
    double inflation_radius = 0.8;
    double cost_scaling_factor = 10.0;

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

    ptr_costmap_params_->setParam(inf_layer_name + ".enabled", true);
    ptr_costmap_params_->setParam(inf_layer_name + ".inflation_radius",
                                  inflation_radius);
    ptr_costmap_params_->setParam(inf_layer_name + ".cost_scaling_factor",
                                  cost_scaling_factor);
    layer_param_.addLayer(CVTE_BABOT::InflationLayerType, inf_layer_name);

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

    auto builder = std::make_shared<Costmap2dBuilder>(ptr_layered_costmap_);
    auto director = std::make_shared<Costmap2dDirector>(builder);
    director->buildCostmap(layer_param_);

    robot_current_pose_.d_x = 0.0;
    robot_current_pose_.d_y = 0.0;
    robot_current_pose_.d_yaw = 0.0;
    ptr_layered_costmap_->updateMap(robot_current_pose_);

    ptr_costmap_ =
        std::make_shared<Costmap2d>(*(ptr_costmap_mediator_->getCostmap()));
  }

  /**
* @brief 测试说明
* １．测试构造函数和复制
* */

  void construct_test() {
    ptr_costmap_->setCost(2, 2, LETHAL_OBSTACLE);
    ptr_costmap_->setCost(2, 3, LETHAL_OBSTACLE);
    ptr_costmap_->setCost(3, 3, LETHAL_OBSTACLE);
    ptr_costmap_->setCost(3, 4, LETHAL_OBSTACLE);

    printMap(*(ptr_costmap_));
    CostmapModel::getPtrInstance()->setPtrCostmap(ptr_costmap_);
    auto obtacle_cf_ptr = std::make_shared<ObstacleCostFunction>();
    auto obtacle_cf_cp = obtacle_cf_ptr;

    EXPECT_NE(nullptr, obtacle_cf_ptr.get());
    EXPECT_NE(nullptr, obtacle_cf_cp.get());
  }

  /**
* @brief 测试说明
* １．测试机器人footprint的代价值计算
* */
  void footprintcost_test() {
    ptr_costmap_->setCost(2, 2, LETHAL_OBSTACLE);
    ptr_costmap_->setCost(2, 3, LETHAL_OBSTACLE);
    ptr_costmap_->setCost(3, 3, LETHAL_OBSTACLE);
    ptr_costmap_->setCost(3, 4, LETHAL_OBSTACLE);
    printMap(*(ptr_costmap_));
    CostmapModel::getPtrInstance()->setPtrCostmap(ptr_costmap_);
    auto obtacle_cf_ptr = std::make_shared<ObstacleCostFunction>();
    std::array<Pose2d, 4> footprint_points;
    CostmapModel::getPtrInstance()->setPtrCostmap(ptr_costmap_);
    auto world_m_ptr = CostmapModel::getPtrInstance();
    typedef struct {
      double x;
      double y;
    } test_param;

    test_param test_table[] = {{4, 2}, {4, 4}, {6, 2}, {6, 4}};
    unsigned int index = 0;
    for (auto value : test_table) {
      Pose2d point;
      point.setPose(value.x, value.y, 0);
      // std::cout << point.x << " " << point.y << std::endl;
      footprint_points[index] = point;
      index++;
    }
    EXPECT_EQ(LETHAL_OBSTACLE,
              obtacle_cf_ptr->footprintCost(3, 3, 0, footprint_points));
    EXPECT_EQ(LETHAL_OBSTACLE,
              obtacle_cf_ptr->footprintCost(3, 4, 0, footprint_points));
    EXPECT_EQ(-6, obtacle_cf_ptr->footprintCost(5, 3, 0, footprint_points));
    EXPECT_EQ(-6, obtacle_cf_ptr->footprintCost(-1.5, -1, 0, footprint_points));
    EXPECT_EQ(-6, obtacle_cf_ptr->footprintCost(-1.5, 3, 0, footprint_points));
  }

  /**
* @brief 测试说明
* １．测试分割因子
* */
  void scaling_factor_test() {
    CostmapModel::getPtrInstance()->setPtrCostmap(ptr_costmap_);
    auto obtacle_cf_ptr = std::make_shared<ObstacleCostFunction>();

    auto traject = std::make_shared<Trajectory>(1.0, 1.0, 0.0, 0.05, 10);

    traject->addPoint(1.0, 1.0, 0.0, 1.0, 1.0, 0.0);
    traject->addPoint(2.0, 2.0, 0.0, 2.0, 2.0, 0.0);
    traject->addPoint(3.0, 3.0, 0.0, 3.0, 3.0, 0.0);
    traject->addPoint(4.0, 4.0, 0.0, 4.0, 4.0, 0.0);
    traject->addPoint(5.0, 5.0, 0.0, 5.0, 5.0, 0.0);
    EXPECT_NE(1.0, obtacle_cf_ptr->getScalingFactor(*traject, 0.2, 2.0, 0.25));
    EXPECT_EQ(1.0, obtacle_cf_ptr->getScalingFactor(*traject, 1.5, 1.0, 0.25));
  }

 private:
  std::shared_ptr<LayeredCostmap> ptr_layers_ = nullptr;
  std::shared_ptr<CostmapMediator> ptr_costmap_mediator_;
  std::shared_ptr<CostmapParameters> ptr_costmap_params_;
  LayerParammeter layer_param_;
  std::shared_ptr<LayeredCostmap> ptr_layered_costmap_;
  std::shared_ptr<Costmap2d> ptr_costmap_;
  WorldmapPose robot_current_pose_;
};

TEST(ObstacleCostFunTest, construct_test) {
  ObstacleCostFunctionTest ObCostTest;
  ObCostTest.construct_test();
}

TEST(ObstacleCostFunTest, footprintcost_test) {
  ObstacleCostFunctionTest ObCostTest;
  ObCostTest.footprintcost_test();
}

TEST(ObstacleCostFunTest, scaling_factor_test) {
  ObstacleCostFunctionTest ObCostTest;
  ObCostTest.scaling_factor_test();
}

}  // namespace CVTE_BABOT

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

#endif