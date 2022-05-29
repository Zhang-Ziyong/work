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

#include <gtest/gtest.h>
#include "costmap_2d.hpp"

namespace CVTE_BABOT {

class Costmap2dTester : public testing::Test {
 public:
  virtual void TestBody() {}

  void costmap2dTest() {
    typedef struct {
      unsigned int size_x;
      unsigned int size_y;
      double reselution;
      double origin_x;
      double origin_y;
      unsigned char value;
    } test_params;

    test_params test[] = {{1000, 1000, 0.005, 0.0, 0.0, 255},
                          {100, 100, -0.005, 0.0, 0.0, 0},
                          {10, 10, 0.5, -50, -50, 0},
                          {100, 100, 0.5, 1000, -1000, 0}};

    for (auto data : test) {
      auto costmap2d =
          std::make_shared<Costmap2d>(data.size_x, data.size_y, data.reselution,
                                      data.origin_x, data.origin_y, data.value);

      EXPECT_EQ(data.size_x, costmap2d->getSizeInCellsX());
      EXPECT_EQ(data.size_y, costmap2d->getSizeInCellsY());
      EXPECT_DOUBLE_EQ(fabs(data.reselution), costmap2d->getResolution());
      EXPECT_DOUBLE_EQ(data.origin_x, costmap2d->getOriginX());
      EXPECT_DOUBLE_EQ(data.origin_y, costmap2d->getOriginY());

      auto ptr_uc_costmap = costmap2d->getCharMap();

      for (unsigned int i = 0; i < data.size_x * data.size_y; ++i) {
        EXPECT_EQ(data.value, static_cast<int>(ptr_uc_costmap[i]));
      }
    }
  }

  void worldToMapTest() {
    auto costmap2d = std::make_shared<Costmap2d>(100, 100, 0.05, 0.0, 0.0, 255);

    typedef struct {
      WorldmapPoint wp_point;
      CostmapPoint cp_point;
      bool result;
    } test_data;

    test_data test[] = {
        {{-1.0, 1.0}, {0, 0}, false},  {{1.0, -1.0}, {0, 0}, false},
        {{-1.0, -1.0}, {0, 0}, false}, {{1.0, 1.0}, {20, 20}, true},
        {{6.0, 1.0}, {0, 0}, false},   {{1.0, 6.0}, {0, 0}, false},
        {{6.0, 6.0}, {0, 0}, false}};

    for (auto data : test) {
      CostmapPoint cp_point;
      if (data.result) {
        EXPECT_TRUE(costmap2d->worldToMap(data.wp_point, cp_point));
        EXPECT_EQ(data.cp_point.ui_x, cp_point.ui_x);
        EXPECT_EQ(data.cp_point.ui_y, cp_point.ui_y);
      } else {
        EXPECT_FALSE(costmap2d->worldToMap(data.wp_point, cp_point));
      }
    }

    typedef struct {
      WorldmapPoint wp_point;
      int cp_x;
      int cp_y;
    } test_data1;

    test_data1 test_1[] = {{{-6.0, -6.0}, -120, -120},
                           {{1.0, 1.0}, 20, 20},
                           {{6.0, 1.0}, 120, 20},
                           {{1.0, 6.0}, 20, 120},
                           {{6.0, 6.0}, 120, 120}};

    for (auto data : test_1) {
      int x, y;
      costmap2d->worldToMapNoBounds(data.wp_point.d_x, data.wp_point.d_y, x, y);
      EXPECT_EQ(data.cp_x, x);
      EXPECT_EQ(data.cp_y, y);
    }

    test_data1 test_2[] = {{{-6.0, -6.0}, 0, 0},
                           {{1.0, 1.0}, 20, 20},
                           {{6.0, 1.0}, 99, 20},
                           {{1.0, 6.0}, 20, 99},
                           {{6.0, 6.0}, 99, 99}};

    for (auto data : test_2) {
      int x, y;
      costmap2d->worldToMapEnforceBounds(data.wp_point.d_x, data.wp_point.d_y,
                                         x, y);
      EXPECT_EQ(data.cp_x, x);
      EXPECT_EQ(data.cp_y, y);
    }

    typedef struct {
      CostmapPoint cm_point;
      WorldmapPoint wm_point;
    } test_data2;

    test_data2 test_3[] = {{
                               {20, 20}, {1.0, 1.0},
                           },
                           {
                               {120, 20}, {6.0, 1.0},
                           },
                           {
                               {20, 120}, {1.0, 6.0},
                           },
                           {
                               {120, 120}, {6.0, 6.0},
                           }};

    for (auto data : test_3) {
      WorldmapPoint wm_point;
      costmap2d->mapToWorld(data.cm_point, wm_point);
      EXPECT_DOUBLE_EQ(data.wm_point.d_x + 0.025, wm_point.d_x);
      EXPECT_DOUBLE_EQ(data.wm_point.d_y + 0.025, wm_point.d_y);
    }
  }

  void updateOriginTest() {
    auto costmap2d = std::make_shared<Costmap2d>(10, 10, 1, 0.0, 0.0, 0);

    for (unsigned int i = 5; i < 10; ++i) {
      for (unsigned int j = 5; j < 10; ++j) {
        costmap2d->setCost(i, j, 255);
      }
    }

    costmap2d->updateOrigin(5, 5);

    for (unsigned int j = 0; j < 10; ++j) {
      for (unsigned int i = 0; i < 10; ++i) {
        int value = static_cast<int>(costmap2d->getCost(i, j));
        if ((i < 5) && (j < 5)) {
          EXPECT_EQ(255, value);
        } else {
          EXPECT_EQ(0, value);
        }
      }
    }
  }

  void setConvexPolygonCostTest() {
    auto costmap2d = std::make_shared<Costmap2d>(10, 10, 1, 0.0, 0.0, 0);
    std::vector<WorldmapPoint> polygon;
    polygon.push_back({2.0, 2.0});
    polygon.push_back({2.0, 8.0});
    polygon.push_back({8.0, 8.0});
    polygon.push_back({8.0, 2.0});

    EXPECT_TRUE(costmap2d->setConvexPolygonCost(polygon, 3));
    for (unsigned int j = 0; j < 10; ++j) {
      for (unsigned int i = 0; i < 10; ++i) {
        int value = static_cast<int>(costmap2d->getCost(i, j));
        if ((i >= 2) && (i < 9) && (j >= 2) && (j < 9)) {
          EXPECT_EQ(3, value);
        } else {
          EXPECT_EQ(0, value);
        }
      }
    }

    std::vector<WorldmapPoint> polygon_1;
    polygon_1.push_back({2.0, 2.0});
    polygon_1.push_back({2.0, 8.0});
    EXPECT_FALSE(costmap2d->setConvexPolygonCost(polygon_1, 3));

    std::vector<WorldmapPoint> polygon_2;
    polygon_2.push_back({100.0, 2.0});
    polygon_2.push_back({2.0, 8.0});
    polygon_2.push_back({8.0, 100.0});
    polygon_2.push_back({8.0, 2.0});
    EXPECT_FALSE(costmap2d->setConvexPolygonCost(polygon_2, 3));
  }

};  // namespace CVTE_BABOT

TEST(Costmap2dTester, costmap2dTest) {
  Costmap2dTester costmap2d_tester;
  costmap2d_tester.costmap2dTest();
}
TEST(Costmap2dTester, worldToMap_test) {
  Costmap2dTester costmap2d_tester;
  costmap2d_tester.worldToMapTest();
}
TEST(LayeredCostmapTester, updateOriginTest) {
  Costmap2dTester costmap2d_tester;
  costmap2d_tester.updateOriginTest();
}

TEST(LayeredCostmapTester, setConvexPolygonCostTest) {
  Costmap2dTester costmap2d_tester;
  costmap2d_tester.setConvexPolygonCostTest();
}

}  // namespace CVTE_BABOT

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
