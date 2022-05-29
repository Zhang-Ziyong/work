/* Software License Agreement (BSD License)
*
* Copyright (c) 2019, CVTE.
* All rights reserved.
*
*@file test_static_layer.cpp
*
*@brief test_static_layer.cpp 文件

*@author wuhuabo <wuhuabo@cvte.com>
*
*@modified wuhuabo <wuhuabo@cvte.com>
*@version current_algo.dev.1.0
*@data 2019-04-18
************************************************************************/
#include <gtest/gtest.h>
#include <memory>
#include "costmap_mediator.hpp"
#include "costmap_testing_helper.hpp"

const unsigned int GRID_WIDTH(10);
const unsigned int GRID_HEIGHT(10);
const double RESOLUTION(1);
const double WINDOW_LENGTH(10);
const unsigned char THRESHOLD(100);
const double RAYTRACE_RANGE(20.0);
const double OBSTACLE_RANGE(20.0);
const double ROBOT_RADIUS(1.0);

void updateFunc(const std::shared_ptr<CVTE_BABOT::WorldmapData> &wm_data,
                std::shared_ptr<CVTE_BABOT::WorldmapData> &ptr_wm_data) {
  ptr_wm_data = std::make_shared<CVTE_BABOT::WorldmapData>(*wm_data);
}

const unsigned char MAP_10_BY_10_CHAR[] = {
    0,   0, 0,   0,   0,   0,  0,  0,   0,   0,   0, 0, 0,   0,   0,
    0,   0, 0,   0,   0,   0,  0,  0,   0,   0,   0, 0, 200, 200, 200,
    0,   0, 0,   0,   100, 0,  0,  200, 200, 200, 0, 0, 0,   0,   100,
    0,   0, 200, 200, 200, 70, 70, 0,   0,   0,   0, 0, 0,   0,   0,
    0,   0, 0,   0,   0,   0,  0,  0,   0,   0,   0, 0, 0,   200, 200,
    200, 0, 0,   0,   0,   0,  0,  0,   0,   0,   0, 0, 255, 255, 255,
    0,   0, 0,   0,   0,   0,  0,  255, 255, 255};

/*
    0,   0,  0,   0,   0,   0,  0, 0,   0,   0,
    0,   0,  0,   0,   0,   0,  0, 0,   0,   0,
    0,   0,  0,   0,   0,   0,  0, 200, 200, 200,
    0,   0,  0,   0,   100, 0,  0, 200, 200, 200,
    0,   0,  0,   0,   100, 0,  0, 200, 200, 200,
    70,  70, 0,   0,   0,   0,  0, 0,   0,   0,
    0,   0,  0,   0,   0,   0,  0, 0,   0,   0,
    0,   0,  0,   200, 200, 200,0, 0,   0,   0,
    0,   0,  0,   0,   0,   0,  0, 255, 255, 255,
    0,   0,  0,   0,   0,   0,  0, 255, 255, 255};
*/

std::vector<unsigned char> MAP_10_BY_10;

bool find(const std::vector<unsigned int> &l, unsigned int n) {
  for (std::vector<unsigned int>::const_iterator it = l.begin(); it != l.end();
       ++it) {
    if (*it == n) return true;
  }

  return false;
}

namespace CVTE_BABOT {
class Static_layer_tester : public testing::Test {
 public:
  virtual void TestBody() {}
  Static_layer_tester() {}

  /**
   *@brief 验证初始化状态是否正确, 异常情况是否返回false
   *
   **/
  void test_initialize() {
    auto ptr_static_layer = std::make_shared<StaticLayer>();
    // CostmapParameters未初始化
    ASSERT_EQ(ptr_static_layer->initialize("static_layer"), false);

    // LayeredCostmap未初始化
    auto ptr_costmap_params = std::make_shared<CostmapParameters>();
    CostmapMediator::getPtrInstance()->setCostmapParameters(ptr_costmap_params);
    ASSERT_EQ(ptr_static_layer->initialize("static_layer"), false);

    // WorldmapData未初始化
    auto ptr_layered_costmap = std::make_shared<LayeredCostmap>(true, true);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);

    ASSERT_EQ(ptr_static_layer->initialize("static_layer"), false);

    std::shared_ptr<WorldmapData> ptr_wm_data;
    CostmapMediator::getPtrInstance()
        ->registerUpdateFunc<std::shared_ptr<WorldmapData>>("static_map",
                                                            ptr_wm_data);
    MAP_10_BY_10.clear();
    for (unsigned int i = 0; i < 10; ++i) {
      for (unsigned int j = 0; j < 10; ++j) {
        MAP_10_BY_10.push_back(MAP_10_BY_10_CHAR[i * 10 + j]);
      }
    }

    ptr_wm_data.reset(new WorldmapData({0, 0}, 10, 10, 1, MAP_10_BY_10));

    CostmapMediator::getPtrInstance()->updateData("static_map", ptr_wm_data);
    ASSERT_EQ(ptr_static_layer->initialize("static_layer"), true);

    ptr_costmap_params->setParam("static_layer.enabled", false);
    ASSERT_EQ(ptr_static_layer->initialize("static_layer"), false);
  }

  /**
   *@brief 测试单个地图值解析是否成功,
   *
   **/
  void test_interpretValue() {
    auto ptr_static_layer = std::make_shared<StaticLayer>();

    auto ptr_costmap_params = std::make_shared<CostmapParameters>();
    CostmapMediator::getPtrInstance()->setCostmapParameters(ptr_costmap_params);

    auto ptr_layered_costmap = std::make_shared<LayeredCostmap>(true, true);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);

    std::shared_ptr<WorldmapData> ptr_wm_data;
    CostmapMediator::getPtrInstance()
        ->registerUpdateFunc<std::shared_ptr<WorldmapData>>("static_map",
                                                            ptr_wm_data);
    MAP_10_BY_10.clear();
    for (unsigned int i = 0; i < 10; ++i) {
      for (unsigned int j = 0; j < 10; ++j) {
        MAP_10_BY_10.push_back(MAP_10_BY_10_CHAR[i * 10 + j]);
      }
    }

    ptr_wm_data.reset(new WorldmapData({0, 0}, 10, 10, 1, MAP_10_BY_10));

    ptr_costmap_params->setParam("static_layer.track_unknown_space", true);
    ptr_costmap_params->setParam("static_layer.lethal_cost_threshold", 100);
    ptr_costmap_params->setParam("static_layer.unknown_cost_value", 255);
    // ptr_costmap_params->setParam("static_layer.lethal_cost_threshold",
    // (int)THRESHOLD);
    ptr_costmap_params->setParam("static_layer.trinary_costmap", true);

    CostmapMediator::getPtrInstance()->updateData("static_map", ptr_wm_data);
    ASSERT_EQ(ptr_static_layer->initialize("static_layer"),
              true);  // initialize the static_layer

    TestStruct<int, unsigned char> t1[] = {
        {255, NO_INFORMATION},
        // greater than lethal_cost_threshold
        {254, LETHAL_OBSTACLE},
        {100, LETHAL_OBSTACLE},
        {101, LETHAL_OBSTACLE},
        // less than lethal_cost_threshold and trinary_costmap is true
        {99, FREE_SPACE},
        {50, FREE_SPACE},
        {0, FREE_SPACE}};

    for (auto it : t1) {
      EXPECT_EQ(ptr_static_layer->interpretValue(it.value), it.expect_value);
    }

    ptr_costmap_params->setParam("static_layer.track_unknown_space", false);
    ptr_costmap_params->setParam("static_layer.lethal_cost_threshold", 99);
    ptr_costmap_params->setParam("static_layer.unknown_cost_value", 253);
    ptr_costmap_params->setParam("static_layer.trinary_costmap", false);
    ASSERT_EQ(ptr_static_layer->initialize("static_layer"),
              true);  // initialize the static_layer

    TestStruct<int, unsigned char> t2[] = {
        // greater than lethal_cost_threshold
        {255, LETHAL_OBSTACLE},
        // unknown value and track_unknown_space is false
        {253, FREE_SPACE},
        // greater than lethal_cost_threshold
        {252, LETHAL_OBSTACLE},
        {100, LETHAL_OBSTACLE},
        {99, LETHAL_OBSTACLE},
        // less than lethal_cost_threshold and trinary_costmap is false
        {50, 128},
        {0, FREE_SPACE}};

    for (auto it : t2) {
      EXPECT_EQ(ptr_static_layer->interpretValue(it.value), it.expect_value);
    }
  }

  /**
   *@brief 测试地图解析是否正确
   *
   **/
  void test_loadMap() {
    auto ptr_costmap_params = std::make_shared<CostmapParameters>();
    CostmapMediator::getPtrInstance()->setCostmapParameters(ptr_costmap_params);

    auto ptr_layered_costmap = std::make_shared<LayeredCostmap>(true, true);

    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);

    std::shared_ptr<WorldmapData> ptr_wm_data;
    CostmapMediator::getPtrInstance()
        ->registerUpdateFunc<std::shared_ptr<WorldmapData>>("static_map",
                                                            ptr_wm_data);

    MAP_10_BY_10.clear();

    for (unsigned int i = 0; i < 10; ++i) {
      for (unsigned int j = 0; j < 10; ++j) {
        MAP_10_BY_10.push_back(MAP_10_BY_10_CHAR[i * 10 + j]);
      }
    }

    ptr_wm_data.reset(new WorldmapData({0, 0}, 10, 10, 1, MAP_10_BY_10));

    // set resolution the same as static_layer
    ptr_layered_costmap->resizeMap(10, 10, 1, 0, 0);

    CostmapMediator::getPtrInstance()->updateData("static_map", ptr_wm_data);
    ptr_costmap_params->setParam("static_layer.enabled", true);
    ptr_costmap_params->setParam("static_layer.lethal_cost_threshold", 100);
    ptr_costmap_params->setParam("static_layer.track_unknown_space", true);
    auto ptr_static_layer = std::make_shared<StaticLayer>();
    ptr_static_layer->initialize("static_layer");

    // size x and y matching the map
    ASSERT_EQ(ptr_static_layer->getSizeInCellsX(), (unsigned int)10);
    ASSERT_EQ(ptr_static_layer->getSizeInCellsY(), (unsigned int)10);
    printMap(*ptr_static_layer);

    std::vector<unsigned int> occupiedCells;
    std::vector<unsigned int> noInformationCells;

    // Verify the num
    for (unsigned int i = 0; i < 10; ++i) {
      for (unsigned int j = 0; j < 10; ++j) {
        if (ptr_static_layer->getCost(i, j) == LETHAL_OBSTACLE) {
          occupiedCells.push_back(ptr_static_layer->getIndex(i, j));
        }
        if (ptr_static_layer->getCost(i, j) == NO_INFORMATION) {
          noInformationCells.push_back(ptr_static_layer->getIndex(i, j));
        }
      }
    }
    ASSERT_EQ(occupiedCells.size(), (unsigned int)14);
    ASSERT_EQ(noInformationCells.size(), (unsigned int)6);

    // Iterate over all obstacle cells and verify that they are greater than 100
    for (std::vector<unsigned int>::const_iterator it = occupiedCells.begin();
         it != occupiedCells.end(); ++it) {
      unsigned int ind = *it;
      unsigned int x, y;
      ptr_static_layer->indexToCells(ind, x, y);
      ASSERT_EQ(find(occupiedCells, ptr_static_layer->getIndex(x, y)), true);
      ASSERT_EQ(MAP_10_BY_10[ind] >= 100, true);
      ASSERT_EQ(ptr_static_layer->getCost(x, y) >= 100, true);
    }

    // test obstacle blocks
    TestStruct<CostmapPoint, bool> t1[] = {// Block (7, 2) to (9, 4)
                                           {{7, 2}, true},
                                           {{8, 2}, true},
                                           {{9, 2}, true},
                                           {{7, 3}, true},
                                           {{8, 3}, true},
                                           {{9, 3}, true},
                                           {{7, 4}, true},
                                           {{8, 4}, true},
                                           {{9, 4}, true},
                                           // Block (4, 3) to (4, 4)
                                           {{4, 3}, true},
                                           {{4, 4}, true},
                                           // Block (3, 7) to (5, 7)
                                           {{3, 7}, true},
                                           {{4, 7}, true},
                                           {{5, 7}, true}};

    for (auto it : t1) {
      ASSERT_EQ(find(occupiedCells,
                     ptr_static_layer->getIndex(it.value.ui_x, it.value.ui_y)),
                it.expect_value);
    }

    // test no information blocks
    TestStruct<CostmapPoint, bool> t2[] = {// Block (7, 8) to (9, 9)
                                           {{7, 8}, true}, {{8, 8}, true},
                                           {{9, 8}, true}, {{7, 9}, true},
                                           {{8, 9}, true}, {{9, 9}, true}};

    for (auto it : t2) {
      ASSERT_EQ(find(noInformationCells,
                     ptr_static_layer->getIndex(it.value.ui_x, it.value.ui_y)),
                it.expect_value);
    }

    // rolling_window false
    ptr_layered_costmap = std::make_shared<LayeredCostmap>(false, true);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);

    // LayeredCostmap's x and y not matching static_layer
    ASSERT_NE(
        ptr_static_layer->getSizeInCellsX(),
        CostmapMediator::getPtrInstance()->getCostmap()->getSizeInCellsX());
    ASSERT_NE(
        ptr_static_layer->getSizeInCellsY(),
        CostmapMediator::getPtrInstance()->getCostmap()->getSizeInCellsY());

    ptr_static_layer->initialize("static_layer");

    ASSERT_EQ(CostmapMediator::getPtrInstance()->isRolling(), false);

    // LayeredCostmap's x and y be changed to match static_layer
    ASSERT_EQ(
        ptr_static_layer->getSizeInCellsX(),
        CostmapMediator::getPtrInstance()->getCostmap()->getSizeInCellsX());
    ASSERT_EQ(
        ptr_static_layer->getSizeInCellsY(),
        CostmapMediator::getPtrInstance()->getCostmap()->getSizeInCellsY());
  }

  /**
   *@brief 测试地图更新是否正确
   *1.测试四个if分支
   *
   **/
  void test_updateCosts() {
    // 1.rolling window, use_maximum is true.
    auto ptr_costmap_params = std::make_shared<CostmapParameters>();
    CostmapMediator::getPtrInstance()->setCostmapParameters(ptr_costmap_params);

    auto ptr_layered_costmap = std::make_shared<LayeredCostmap>(true, true);

    // set resolution the same as static_layer
    ptr_layered_costmap->resizeMap(200, 200, 1, 0, 0);

    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);

    std::shared_ptr<WorldmapData> ptr_wm_data;
    CostmapMediator::getPtrInstance()
        ->registerUpdateFunc<std::shared_ptr<WorldmapData>>("static_map",
                                                            ptr_wm_data);

    MAP_10_BY_10.clear();

    for (unsigned int i = 0; i < 10; ++i) {
      for (unsigned int j = 0; j < 10; ++j) {
        MAP_10_BY_10.push_back(MAP_10_BY_10_CHAR[i * 10 + j]);
      }
    }

    ptr_wm_data.reset(new WorldmapData({0, 0}, 10, 10, 1, MAP_10_BY_10));

    CostmapMediator::getPtrInstance()->updateData("static_map", ptr_wm_data);
    ptr_costmap_params->setParam("static_layer.enabled", true);
    ptr_costmap_params->setParam("static_layer.use_maximum", true);
    ptr_costmap_params->setParam("static_layer.lethal_cost_threshold", 100);
    ptr_costmap_params->setParam("static_layer.track_unknown_space", true);

    // Verify the num
    std::vector<unsigned int> occupiedCells;
    std::vector<unsigned int> noInformationCells;

    auto ptr_costmap =
        std::make_shared<Costmap2d>(*ptr_layered_costmap->getCostmap());
    for (unsigned int i = 0; i < ptr_costmap->getSizeInCellsY(); ++i) {
      for (unsigned int j = 0; j < ptr_costmap->getSizeInCellsX(); ++j) {
        if (ptr_costmap->getCost(i, j) == LETHAL_OBSTACLE) {
          occupiedCells.push_back(ptr_costmap->getIndex(i, j));
        }
        if (ptr_costmap->getCost(i, j) == NO_INFORMATION) {
          noInformationCells.push_back(ptr_costmap->getIndex(i, j));
        }
      }
    }

    // all no information
    ASSERT_EQ(occupiedCells.size(), 0);
    ASSERT_EQ(noInformationCells.size(),
              ptr_costmap->getSizeInCellsX() * ptr_costmap->getSizeInCellsY());

    auto ptr_static_layer = std::make_shared<StaticLayer>();
    ptr_static_layer->initialize("static_layer");

    ASSERT_NE(
        ptr_static_layer->getSizeInCellsX(),
        CostmapMediator::getPtrInstance()->getCostmap()->getSizeInCellsX());
    ASSERT_NE(
        ptr_static_layer->getSizeInCellsY(),
        CostmapMediator::getPtrInstance()->getCostmap()->getSizeInCellsY());

    ptr_static_layer->updateCosts(ptr_costmap, 0, 0, 5, 5);

    occupiedCells.clear();
    noInformationCells.clear();
    for (unsigned int i = 0; i < ptr_costmap->getSizeInCellsY(); ++i) {
      for (unsigned int j = 0; j < ptr_costmap->getSizeInCellsX(); ++j) {
        if (ptr_costmap->getCost(i, j) == LETHAL_OBSTACLE) {
          occupiedCells.push_back(ptr_costmap->getIndex(i, j));
        }
        if (ptr_costmap->getCost(i, j) == NO_INFORMATION) {
          noInformationCells.push_back(ptr_costmap->getIndex(i, j));
        }
      }
    }

    // remain max value: NO_INFORMATION
    ASSERT_EQ(occupiedCells.size(), 0);
    ASSERT_EQ(noInformationCells.size(),
              ptr_costmap->getSizeInCellsX() * ptr_costmap->getSizeInCellsY());

    // 2.rolling window, use_maximum is false.
    ptr_static_layer->use_maximum_ = false;
    ptr_static_layer->updateCosts(ptr_costmap, 0, 0, 5, 5);

    occupiedCells.clear();
    noInformationCells.clear();
    for (unsigned int i = 0; i < ptr_costmap->getSizeInCellsY(); ++i) {
      for (unsigned int j = 0; j < ptr_costmap->getSizeInCellsX(); ++j) {
        if (ptr_costmap->getCost(i, j) == LETHAL_OBSTACLE) {
          occupiedCells.push_back(ptr_costmap->getIndex(i, j));
        }
        if (ptr_costmap->getCost(i, j) == NO_INFORMATION) {
          noInformationCells.push_back(ptr_costmap->getIndex(i, j));
        }
      }
    }

    ASSERT_EQ(occupiedCells.size(), 2);
    ASSERT_EQ(
        noInformationCells.size(),
        ptr_costmap->getSizeInCellsX() * ptr_costmap->getSizeInCellsY() - 25);

    // 3.not rolling window, use_maximum is false.
    ptr_layered_costmap = std::make_shared<LayeredCostmap>(false, true);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);

    ptr_costmap =
        std::make_shared<Costmap2d>(*ptr_layered_costmap->getCostmap());
    ptr_static_layer->initialize("static_layer");
    ptr_static_layer->use_maximum_ = false;

    ptr_static_layer->updateCosts(ptr_costmap, 0, 0, 5, 5);

    occupiedCells.clear();
    noInformationCells.clear();
    for (unsigned int i = 0; i < ptr_costmap->getSizeInCellsY(); ++i) {
      for (unsigned int j = 0; j < ptr_costmap->getSizeInCellsX(); ++j) {
        if (ptr_costmap->getCost(i, j) == LETHAL_OBSTACLE) {
          occupiedCells.push_back(ptr_costmap->getIndex(i, j));
        }
        if (ptr_costmap->getCost(i, j) == NO_INFORMATION) {
          noInformationCells.push_back(ptr_costmap->getIndex(i, j));
        }
      }
    }

    ASSERT_EQ(occupiedCells.size(), 2);
    ASSERT_EQ(noInformationCells.size(), 75);

    // 4.not rolling window, use_maximum is true.
    ptr_layered_costmap = std::make_shared<LayeredCostmap>(false, true);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);

    ptr_costmap =
        std::make_shared<Costmap2d>(*ptr_layered_costmap->getCostmap());
    ptr_static_layer->initialize("static_layer");

    ptr_static_layer->use_maximum_ = true;
    ptr_static_layer->updateCosts(ptr_costmap, 3, 7, 10, 10);

    occupiedCells.clear();
    noInformationCells.clear();
    for (unsigned int i = 0; i < ptr_costmap->getSizeInCellsY(); ++i) {
      for (unsigned int j = 0; j < ptr_costmap->getSizeInCellsX(); ++j) {
        if (ptr_costmap->getCost(i, j) == LETHAL_OBSTACLE) {
          occupiedCells.push_back(ptr_costmap->getIndex(i, j));
        }
        if (ptr_costmap->getCost(i, j) == NO_INFORMATION) {
          noInformationCells.push_back(ptr_costmap->getIndex(i, j));
        }
      }
    }
    printMap(*ptr_costmap);
    ASSERT_EQ(occupiedCells.size(), 3);
    ASSERT_EQ(
        noInformationCells.size(),
        ptr_costmap->getSizeInCellsX() * ptr_costmap->getSizeInCellsY() - 15);
  }

  /**
   *@brief 测试计算更新范围是否正确
   *1.分是否rolling window两种情况测试
   **/
  void test_updateBounds() {
    // 1.rolling window
    auto ptr_costmap_params = std::make_shared<CostmapParameters>();
    CostmapMediator::getPtrInstance()->setCostmapParameters(ptr_costmap_params);

    auto ptr_layered_costmap = std::make_shared<LayeredCostmap>(true, true);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);

    std::shared_ptr<WorldmapData> ptr_wm_data;
    CostmapMediator::getPtrInstance()
        ->registerUpdateFunc<std::shared_ptr<WorldmapData>>("static_map",
                                                            ptr_wm_data);

    MAP_10_BY_10.clear();

    for (unsigned int i = 0; i < 10; ++i) {
      for (unsigned int j = 0; j < 10; ++j) {
        MAP_10_BY_10.push_back(MAP_10_BY_10_CHAR[i * 10 + j]);
      }
    }

    ptr_wm_data.reset(new WorldmapData({0, 0}, 10, 10, 1, MAP_10_BY_10));

    CostmapMediator::getPtrInstance()->updateData("static_map", ptr_wm_data);
    ptr_costmap_params->setParam("static_layer.enabled", true);
    ptr_costmap_params->setParam("static_layer.use_maximum", true);
    ptr_costmap_params->setParam("static_layer.lethal_cost_threshold", 100);
    ptr_costmap_params->setParam("static_layer.track_unknown_space", true);

    auto ptr_static_layer = std::make_shared<StaticLayer>();
    ptr_static_layer->initialize("static_layer");

    WorldmapPose wp_robot_pose = {0, 0, 0};
    CostmapBound cb_costmap_bound = {1e30, 1e30, -1e30, -1e30};
    ptr_static_layer->updateBounds(wp_robot_pose, cb_costmap_bound);

    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_min_x, 0.5);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_min_y, 0.5);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_max_x, 10.5);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_max_y, 10.5);

    // update again
    cb_costmap_bound = {1e30, 1e30, -1e30, -1e30};
    ptr_static_layer->updateBounds(wp_robot_pose, cb_costmap_bound);

    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_min_x, 0.5);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_min_y, 0.5);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_max_x, 10.5);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_max_y, 10.5);

    //  2.not rolling window, the static_layer will only updateBounds once
    //  until there is new map incoming.
    ptr_layered_costmap = std::make_shared<LayeredCostmap>(false, true);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);

    ptr_static_layer->initialize("static_layer");
    cb_costmap_bound = {1e30, 1e30, -1e30, -1e30};
    ptr_static_layer->updateBounds(wp_robot_pose, cb_costmap_bound);

    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_min_x, 0.5);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_min_y, 0.5);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_max_x, 10.5);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_max_y, 10.5);

    cb_costmap_bound = {1e30, 1e30, -1e30, -1e30};
    ptr_static_layer->updateBounds(wp_robot_pose, cb_costmap_bound);

    // not update again
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_min_x, 1e30);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_min_y, 1e30);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_max_x, -1e30);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_max_y, -1e30);
  }
};  // class

auto ptr_static_layer_tester = std::make_shared<Static_layer_tester>();

TEST(testCase, test_initialize) { ptr_static_layer_tester->test_initialize(); }

TEST(testCase, test_interpretValue) {
  ptr_static_layer_tester->test_interpretValue();
}

TEST(testCase, test_loadMap) { ptr_static_layer_tester->test_loadMap(); }

TEST(testCase, test_updateCosts) {
  ptr_static_layer_tester->test_updateCosts();
}

TEST(testCase, test_updateBounds) {
  ptr_static_layer_tester->test_updateBounds();
}

}  // namespace CVTE_BABOT

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
