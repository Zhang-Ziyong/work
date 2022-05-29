/************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2018, CVTE.
* All rights reserved.
*
*@file test_inflation_layer.cpp
*
*@brief 测试inflation_layer.cpp 文件

*@author chenmingjian <chenmingjian@cvte.com>
*
*@modified chenmingjian <chenmingjian@cvte.com>
*@version current_algo.dev.1.0
*@data 2019-04-17
************************************************************************/

#include <gtest/gtest.h>
#include "costmap_mediator.hpp"
#include "inflation_layer.hpp"

namespace CVTE_BABOT {

class InflationLayerTester : public testing::Test {
 public:
  virtual void TestBody() {}

  void InflationLayerInitTest() {
    auto ptr_inflation_layer = std::make_shared<InflationLayer>();
    EXPECT_FALSE(ptr_inflation_layer->initialize("inflation_layer"));

    auto ptr_costmap_params = std::make_shared<CostmapParameters>();
    CostmapMediator::getPtrInstance()->setCostmapParameters(ptr_costmap_params);
    EXPECT_FALSE(ptr_inflation_layer->initialize("inflation_layer"));

    auto ptr_layered_costmap = std::make_shared<LayeredCostmap>(true, true);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);

    EXPECT_TRUE(ptr_inflation_layer->initialize("inflation_layer"));
  }

  void computeCachesTest() {
    auto ptr_costmap_params = std::make_shared<CostmapParameters>();
    auto ptr_layered_costmap = std::make_shared<LayeredCostmap>(true, true);
    ptr_costmap_params->setParam("inflation_layer.enabled", true);
    ptr_costmap_params->setParam("inflation_layer.inflation_radius", 2.0);

    auto ptr_costmap_mediator = CostmapMediator::getPtrInstance();
    ptr_costmap_mediator->setCostmapParameters(ptr_costmap_params);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);

    ptr_layered_costmap->resizeMap(10, 10, 1, 0, 0);

    auto ptr_inflation_layer = std::make_shared<InflationLayer>();
    EXPECT_TRUE(ptr_inflation_layer->initialize("inflation_layer"));
    ptr_inflation_layer->inflation_radius_ = 2.0;
    ptr_layered_costmap->addLayer(ptr_inflation_layer);

    std::vector<WorldmapPoint> polygon;
    polygon.push_back({-1.0, -1.0});
    polygon.push_back({1.0, -1.0});
    polygon.push_back({1.0, 1.0});
    polygon.push_back({-1.0, 1.0});
    ptr_layered_costmap->setFootprint(polygon);

    for (unsigned int i = 0;
         i <= ptr_inflation_layer->cell_inflation_radius_ + 1; ++i) {
      for (unsigned int j = 0;
           j <= ptr_inflation_layer->cell_inflation_radius_ + 1; ++j) {
        double dist = ptr_inflation_layer->cached_distances_[i][j];
        EXPECT_DOUBLE_EQ(dist, hypot(i, j));
      }
    }

    EXPECT_EQ(254, static_cast<int>(ptr_inflation_layer->cached_costs_[0][0]));
    EXPECT_EQ(253, static_cast<int>(ptr_inflation_layer->cached_costs_[0][1]));
    EXPECT_EQ(253, static_cast<int>(ptr_inflation_layer->cached_costs_[1][0]));
    EXPECT_EQ(31, static_cast<int>(ptr_inflation_layer->cached_costs_[1][1]));
    EXPECT_EQ(1, static_cast<int>(ptr_inflation_layer->cached_costs_[2][0]));
    EXPECT_EQ(1, static_cast<int>(ptr_inflation_layer->cached_costs_[0][2]));
  }

  void updateCostTest() {
    auto ptr_costmap_params = std::make_shared<CostmapParameters>();
    auto ptr_layered_costmap = std::make_shared<LayeredCostmap>(true, true);
    ptr_costmap_params->setParam("inflation_layer.enabled", true);
    ptr_costmap_params->setParam("inflation_layer.inflation_radius", 4.0);

    auto ptr_costmap_mediator = CostmapMediator::getPtrInstance();
    ptr_costmap_mediator->setCostmapParameters(ptr_costmap_params);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);

    ptr_layered_costmap->resizeMap(10, 10, 1, 0, 0);
    auto master_costmap =
        std::make_shared<Costmap2d>(*ptr_layered_costmap->getCostmap());
    master_costmap->setCost(5, 5, LETHAL_OBSTACLE);
    master_costmap->setCost(5, 7, 0);
    auto ptr_inflation_layer = std::make_shared<InflationLayer>();
    EXPECT_TRUE(ptr_inflation_layer->initialize("inflation_layer"));
    ptr_layered_costmap->addLayer(ptr_inflation_layer);

    std::vector<WorldmapPoint> polygon;
    polygon.push_back({-1.0, -1.0});
    polygon.push_back({1.0, -1.0});
    polygon.push_back({1.0, 1.0});
    polygon.push_back({-1.0, 1.0});
    ptr_layered_costmap->setFootprint(polygon);

    ptr_inflation_layer->updateCosts(master_costmap, 0, 0, 9, 9);

    EXPECT_EQ(254, static_cast<int>(master_costmap->getCost(5, 5)));
    EXPECT_EQ(1, static_cast<int>(master_costmap->getCost(5, 7)));
    EXPECT_EQ(253, static_cast<int>(master_costmap->getCost(5, 4)));
    EXPECT_EQ(253, static_cast<int>(master_costmap->getCost(5, 6)));
    EXPECT_EQ(253, static_cast<int>(master_costmap->getCost(4, 5)));
    EXPECT_EQ(253, static_cast<int>(master_costmap->getCost(6, 5)));
  }

  void updateBound() {
    auto ptr_costmap_params = std::make_shared<CostmapParameters>();
    auto ptr_layered_costmap = std::make_shared<LayeredCostmap>(true, true);
    ptr_costmap_params->setParam("inflation_layer.enabled", true);
    ptr_costmap_params->setParam("inflation_layer.inflation_radius", 4.0);

    auto ptr_costmap_mediator = CostmapMediator::getPtrInstance();
    ptr_costmap_mediator->setCostmapParameters(ptr_costmap_params);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);

    ptr_layered_costmap->resizeMap(10, 10, 1, 0, 0);
    auto master_costmap =
        std::make_shared<Costmap2d>(*ptr_layered_costmap->getCostmap());
    master_costmap->setCost(5, 5, LETHAL_OBSTACLE);
    master_costmap->setCost(5, 7, 0);
    auto ptr_inflation_layer = std::make_shared<InflationLayer>();
    EXPECT_TRUE(ptr_inflation_layer->initialize("inflation_layer"));
    ptr_layered_costmap->addLayer(ptr_inflation_layer);

    std::vector<WorldmapPoint> polygon;
    polygon.push_back({-1.0, -1.0});
    polygon.push_back({1.0, -1.0});
    polygon.push_back({1.0, 1.0});
    polygon.push_back({-1.0, 1.0});
    ptr_layered_costmap->setFootprint(polygon);

    WorldmapPose wp_robot_pose = {0, 0, 0};
    CostmapBound cb_costmap_bound = {1e30, 1e30, -1e30, -1e30};

    ptr_inflation_layer->updateBounds(wp_robot_pose, cb_costmap_bound);

    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_min_x,
                     -std::numeric_limits<float>::max());
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_min_y,
                     -std::numeric_limits<float>::max());
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_max_x,
                     std::numeric_limits<float>::max());
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_max_y,
                     std::numeric_limits<float>::max());

    cb_costmap_bound = {1, 2, 7, 8};

    ptr_inflation_layer->updateBounds(wp_robot_pose, cb_costmap_bound);

    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_min_x, -3);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_min_y, -2);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_max_x, 11);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_max_y, 12);

    cb_costmap_bound = {0, 0, 0, 0};

    ptr_inflation_layer->updateBounds(wp_robot_pose, cb_costmap_bound);

    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_min_x, -4);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_min_y, -4);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_max_x, 11);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_max_y, 12);
  }
};

TEST(InflationLayerTester, InflationLayerTest) {
  InflationLayerTester inflation_layer_tester;
  inflation_layer_tester.InflationLayerInitTest();
}

TEST(InflationLayerTester, computeCachesTest) {
  InflationLayerTester inflation_layer_tester;
  inflation_layer_tester.computeCachesTest();
}

TEST(InflationLayerTester, updateCostTest) {
  InflationLayerTester inflation_layer_tester;
  inflation_layer_tester.updateCostTest();
}

TEST(InflationLayerTester, updateBound) {
  InflationLayerTester inflation_layer_tester;
  inflation_layer_tester.updateBound();
}

}  // namespace CVTE_BABOT
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
