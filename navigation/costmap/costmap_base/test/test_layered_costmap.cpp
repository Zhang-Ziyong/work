/************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2018, CVTE.
* All rights reserved.
*
*@file test_layered_costmap.cpp
*
*@brief 测试layered_costmap.cpp 文件

*@author chenmingjian <chenmingjian@cvte.com>
*
*@modified chenmingjian <chenmingjian@cvte.com>
*@version current_algo.dev.1.0
*@data 2019-04-16
************************************************************************/

#include "layered_costmap.hpp"
#include <gtest/gtest.h>

namespace CVTE_BABOT {

class LayeredCostmapTester : public testing::Test {
public:
  virtual void TestBody() {}

  void layeredCostmapTest() {
    bool rolling_window = true;
    bool track_unknow = true;

    auto ptr_layered_costmap =
        std::make_shared<LayeredCostmap>(rolling_window, track_unknow);
    auto costmap = ptr_layered_costmap->getCostmap();

    EXPECT_EQ(200, costmap->getSizeInCellsX());
    EXPECT_EQ(200, costmap->getSizeInCellsY());
    EXPECT_DOUBLE_EQ(0.05, costmap->getResolution());
    EXPECT_DOUBLE_EQ(-5.0, costmap->getOriginX());
    EXPECT_DOUBLE_EQ(-5.0, costmap->getOriginY());
    EXPECT_EQ(255, static_cast<int>(costmap->getDefaultValue()));
    EXPECT_TRUE(ptr_layered_costmap->isRolling());

    rolling_window = false;
    track_unknow = false;

    auto ptr_layered_costmap_1 =
        std::make_shared<LayeredCostmap>(rolling_window, track_unknow);
    auto costmap_1 = ptr_layered_costmap_1->getCostmap();

    EXPECT_EQ(200, costmap_1->getSizeInCellsX());
    EXPECT_EQ(200, costmap_1->getSizeInCellsY());
    EXPECT_DOUBLE_EQ(0.05, costmap_1->getResolution());
    EXPECT_DOUBLE_EQ(-5.0, costmap_1->getOriginX());
    EXPECT_DOUBLE_EQ(-5.0, costmap_1->getOriginY());
    EXPECT_EQ(0, static_cast<int>(costmap_1->getDefaultValue()));
    EXPECT_FALSE(ptr_layered_costmap_1->isRolling());
  }

  void resizeMapTest() {

    bool rolling_window = true;
    bool track_unknow = false;

    typedef struct {
      unsigned int size_x;
      unsigned int size_y;
      double reselution;
      double origin_x;
      double origin_y;
      bool result;
    } test_params;

    test_params test[] = {{1000, 1000, 0.005, 0.0, 0.0, true},
                          {100, 100, -0.005, 0.0, 0.0, true},
                          {10, 10, 0.005, -50, -50, true},
                          {100, 100, 0.005, 1000, -1000, true}};

    for (auto data : test) {
      auto ptr_layered_costmap =
          std::make_shared<LayeredCostmap>(rolling_window, track_unknow);

      if (data.result) {

        EXPECT_TRUE(ptr_layered_costmap->resizeMap(
            data.size_x, data.size_y, data.reselution, data.origin_x,
            data.origin_y));

        auto costmap = ptr_layered_costmap->getCostmap();

        EXPECT_EQ(data.size_x, costmap->getSizeInCellsX());
        EXPECT_EQ(data.size_y, costmap->getSizeInCellsY());
        EXPECT_DOUBLE_EQ(fabs(data.reselution), costmap->getResolution());
        EXPECT_DOUBLE_EQ(data.origin_x, costmap->getOriginX());
        EXPECT_DOUBLE_EQ(data.origin_y, costmap->getOriginY());
      } else {
        EXPECT_FALSE(ptr_layered_costmap->resizeMap(
            data.size_x, data.size_y, data.reselution, data.origin_x,
            data.origin_y));
      }
    }
  }

  void getupdateMapTest() {

    // TODO(icanchen):依赖其它的类，需要其它类测试完成在测试该函数。
    bool rolling_window = true;
    bool track_unknow = true;

    auto ptr_layered_costmap =
        std::make_shared<LayeredCostmap>(rolling_window, track_unknow);
    auto costmap = ptr_layered_costmap->getCostmap();

    ptr_layered_costmap->resizeMap(10, 10, 1, 50, 50);
  }
};

TEST(LayeredCostmapTester, init_test) {
  LayeredCostmapTester layered_costmap_tester;
  layered_costmap_tester.layeredCostmapTest();
}
TEST(LayeredCostmapTester, resizeMap_test) {
  LayeredCostmapTester layered_costmap_tester;
  layered_costmap_tester.resizeMapTest();
}
TEST(LayeredCostmapTester, getupdateMap_test) {
  LayeredCostmapTester layered_costmap_tester;
  layered_costmap_tester.getupdateMapTest();
}

} // namespace CVTE_BABOT

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
