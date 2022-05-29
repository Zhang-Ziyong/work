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

namespace CVTE_BABOT {
void ASSERT_EQ_BOUND(const CostmapBound &lb, const CostmapBound &rb) {
  ASSERT_DOUBLE_EQ(lb.d_min_x, rb.d_min_x);
  ASSERT_DOUBLE_EQ(lb.d_min_y, rb.d_min_y);
  ASSERT_DOUBLE_EQ(lb.d_max_x, rb.d_max_x);
  ASSERT_DOUBLE_EQ(lb.d_max_y, rb.d_max_y);
}

class voxel_layer_tester : public testing::Test {
 public:
  virtual void TestBody() {}
  voxel_layer_tester() {}

  void test_initialize() {
    auto ptr_voxel_layer = std::make_shared<VoxelLayer>();

    auto ptr_costmap_params = std::make_shared<CostmapParameters>();
    CostmapMediator::getPtrInstance()->setCostmapParameters(ptr_costmap_params);

    auto ptr_layered_costmap = std::make_shared<LayeredCostmap>(true, false);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);
    ptr_costmap_params->setParam("voxel_layer.enabled", true);
    ptr_costmap_params->setParam("voxel_layer.marking_cloud_sources",
                                 std::string("scan back_scan"));
    ptr_costmap_params->setParam("voxel_layer.clearing_cloud_sources",
                                 std::string("scan front_scan"));
    ptr_costmap_params->setParam("voxel_layer.max_obstacle_height", 0.8);
    ptr_costmap_params->setParam("voxel_layer.min_obstacle_height", 0.1);
    ptr_costmap_params->setParam("voxel_layer.z_voxels", 10);
    ptr_costmap_params->setParam("voxel_layer.unknown_threshold", 5);
    ASSERT_EQ(ptr_voxel_layer->initialize("voxel_layer"), true);

    // two member
    ASSERT_EQ(ptr_voxel_layer->v_marking_cloud_names_.size(), 2);
    ASSERT_EQ(ptr_voxel_layer->v_clearing_cloud_names_.size(), 2);

    ASSERT_DOUBLE_EQ(ptr_voxel_layer->d_max_obstacle_height_, 0.8);
    ASSERT_DOUBLE_EQ(ptr_voxel_layer->d_min_obstacle_height_, 0.1);
    ASSERT_EQ(ptr_voxel_layer->ui_size_z_, 10);
    ASSERT_EQ(ptr_voxel_layer->unknown_threshold_, 11);

    std::vector<std::shared_ptr<const CostmapCloud>> v_clouds,
        v_clearing_clouds;
    ptr_voxel_layer->getMarkingClouds(v_clouds);
    ptr_voxel_layer->getClearingClouds(v_clearing_clouds);

    // no data.
    ASSERT_EQ(v_clouds.size(), 0);
    ASSERT_EQ(v_clearing_clouds.size(), 0);

    CostmapMediator::getPtrInstance()->updateData(
        "scan", std::make_shared<CostmapCloud>());

    ptr_voxel_layer->getMarkingClouds(v_clouds);
    ptr_voxel_layer->getClearingClouds(v_clearing_clouds);
    ASSERT_EQ(v_clouds.size(), 1);
    ASSERT_EQ(v_clearing_clouds.size(), 1);

    v_clouds.clear();
    v_clearing_clouds.clear();

    CostmapMediator::getPtrInstance()->updateData(
        "back_scan", std::make_shared<CostmapCloud>());
    ptr_voxel_layer->getMarkingClouds(v_clouds);
    ptr_voxel_layer->getClearingClouds(v_clearing_clouds);

    ASSERT_EQ(v_clouds.size(), 2);
    ASSERT_EQ(v_clearing_clouds.size(), 1);
  }

  void test_updateRaytraceBounds() {
    auto ptr_voxel_layer = std::make_shared<VoxelLayer>();

    auto ptr_costmap_params = std::make_shared<CostmapParameters>();
    CostmapMediator::getPtrInstance()->setCostmapParameters(ptr_costmap_params);

    auto ptr_layered_costmap = std::make_shared<LayeredCostmap>(true, false);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);
    ptr_costmap_params->setParam("voxel_layer.enabled", true);

    ASSERT_EQ(ptr_voxel_layer->initialize("voxel_layer"), true);

    WorldmapPoint wp_origin{0.0, 0.0};
    WorldmapPoint wp_point{3.0, 4.0};
    double range = 2.5;
    CostmapBound cb_costmap_bound = {1.0, 1.0, 2.0, 3.0};

    // greater than range
    ptr_voxel_layer->updateRaytraceBounds(wp_origin, wp_point, range,
                                          cb_costmap_bound);
    CostmapBound bound = {1.0, 1.0, 2.0, 3.0};
    ASSERT_EQ_BOUND(cb_costmap_bound, bound);

    range = 5.0;
    cb_costmap_bound = {6.0, 7.0, 10.0, 10.0};
    // update min
    ptr_voxel_layer->updateRaytraceBounds(wp_origin, wp_point, range,
                                          cb_costmap_bound);
    bound = {3.0, 4.0, 10.0, 10.0};
    ASSERT_EQ_BOUND(cb_costmap_bound, bound);

    cb_costmap_bound = {0.0, 0.0, 2.0, 3.0};
    // update max
    ptr_voxel_layer->updateRaytraceBounds(wp_origin, wp_point, range,
                                          cb_costmap_bound);
    bound = {0.0, 0.0, 3.0, 4.0};
    ASSERT_EQ_BOUND(cb_costmap_bound, bound);

    range = 5.0;
    wp_origin = {-0.5, -0.5};
    cb_costmap_bound = {0.0, 0.0, 2.0, 3.0};
    wp_point = {2.5, 3.5};
    // update max, not considering origin
    ptr_voxel_layer->updateRaytraceBounds(wp_origin, wp_point, range,
                                          cb_costmap_bound);
    bound = {0.0, 0.0, 2.5, 3.5};
    ASSERT_EQ_BOUND(cb_costmap_bound, bound);
  }

  void test_raytraceFreespace() {
    auto ptr_voxel_layer = std::make_shared<VoxelLayer>();

    auto ptr_costmap_params = std::make_shared<CostmapParameters>();
    CostmapMediator::getPtrInstance()->setCostmapParameters(ptr_costmap_params);

    auto ptr_layered_costmap = std::make_shared<LayeredCostmap>(true, false);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);
    ptr_costmap_params->setParam("voxel_layer.enabled", true);
    ptr_costmap_params->setParam("voxel_layer.marking_cloud_sources",
                                 std::string("scan back_scan"));
    ptr_costmap_params->setParam("voxel_layer.clearing_cloud_sources",
                                 std::string("scan front_scan"));
    ptr_costmap_params->setParam("voxel_layer.max_obstacle_height", 0.8);
    ptr_costmap_params->setParam("voxel_layer.min_obstacle_height", -0.1);

    ASSERT_EQ(ptr_voxel_layer->initialize("voxel_layer"), true);

    CostmapCloud clearing_clouds;
    clearing_clouds.raytrace_range_ = 5.0;
    clearing_clouds.ptr_v_cloud_->push_back({2.5, 3.5, 0.00});

    CostmapBound cb_costmap_bound = {0.00, 0.00, 2.00, 2.00};
    // outside the map
    clearing_clouds.origin_ = {-10.0, -10.0, 0.1};
    ptr_voxel_layer->raytraceFreespace(clearing_clouds, cb_costmap_bound);
    CostmapBound bound = {0.0, 0.0, 2.0, 2.0};
    ASSERT_EQ_BOUND(cb_costmap_bound, bound);

    // outside the map
    clearing_clouds.origin_ = {5.0, 5.0};
    ptr_voxel_layer->raytraceFreespace(clearing_clouds, cb_costmap_bound);
    bound = {0.0, 0.0, 2.0, 2.0};
    ASSERT_EQ_BOUND(cb_costmap_bound, bound);

    clearing_clouds.origin_ = {-0.5, -0.5};
    ptr_voxel_layer->raytraceFreespace(clearing_clouds, cb_costmap_bound);
    bound = {-0.5, -0.5, 2.5, 3.5};
    ASSERT_EQ_BOUND(cb_costmap_bound, bound);

    CostmapCloud clearing_clouds2;
    clearing_clouds2.raytrace_range_ = 5.0;

    CostmapBound cb_costmap_bound2 = {0.00, 0.00, 2.00, 2.00};

    clearing_clouds2.origin_ = {-2.0, -1.0};
    clearing_clouds2.ptr_v_cloud_->push_back({-8.0, -9.0, 0.00});
    ptr_voxel_layer->raytraceFreespace(clearing_clouds2, cb_costmap_bound2);
    // touch min
    bound = {-5.0, -5.0, 2.0, 2.0};
    ASSERT_EQ_BOUND(cb_costmap_bound2, bound);

    clearing_clouds2.origin_ = {1.0, 3.0};
    CostmapBound cb_costmap_bound3 = {0.00, 0.00, 2.00, 2.00};
    ptr_voxel_layer->raytraceFreespace(clearing_clouds2, cb_costmap_bound3);
    // limit the range
    bound = {-2.0, -1.0, 2.0, 3.0};
    ASSERT_EQ_BOUND(cb_costmap_bound3, bound);

    ptr_voxel_layer->setDefaultValue(LETHAL_OBSTACLE);
    ptr_layered_costmap->addLayer(ptr_voxel_layer);
    ptr_layered_costmap->resizeMap(10, 10, 1, 0.0, 0.0);
    CostmapCloud clearing_clouds4;
    clearing_clouds4.raytrace_range_ = 5.0;

    CostmapBound cb_costmap_bound4 = {0.00, 0.00, 2.00, 2.00};

    clearing_clouds4.origin_ = {-0.0, -0.0};
    clearing_clouds4.ptr_v_cloud_->push_back({3.0, 4.0, 0.00});
    clearing_clouds4.ptr_v_cloud_->push_back({4.0, 1.0, 0.00});

    ptr_voxel_layer->raytraceFreespace(clearing_clouds4, cb_costmap_bound4);

    bound = {0.0, 0.0, 4.0, 4.0};
    ASSERT_EQ_BOUND(cb_costmap_bound4, bound);

    printMap(*ptr_voxel_layer);
    ASSERT_EQ(9, countValues(*ptr_voxel_layer, FREE_SPACE));
  }

  void test_updateBounds() {
    auto ptr_voxel_layer = std::make_shared<VoxelLayer>();

    auto ptr_costmap_params = std::make_shared<CostmapParameters>();
    CostmapMediator::getPtrInstance()->setCostmapParameters(ptr_costmap_params);

    auto ptr_layered_costmap = std::make_shared<LayeredCostmap>(true, false);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);
    ptr_costmap_params->setParam("voxel_layer.enabled", true);
    ptr_costmap_params->setParam("voxel_layer.marking_cloud_sources",
                                 std::string("scan back_scan"));
    ptr_costmap_params->setParam("voxel_layer.clearing_cloud_sources",
                                 std::string("scan front_scan"));

    ASSERT_EQ(ptr_voxel_layer->initialize("voxel_layer"), true);

    ptr_voxel_layer->setDefaultValue(NO_INFORMATION);
    ptr_layered_costmap->addLayer(ptr_voxel_layer);
    ptr_layered_costmap->resizeMap(10, 10, 1, 0.0, 0.0);

    // scan
    auto ptr_clearing_clouds = std::make_shared<CostmapCloud>();
    ptr_clearing_clouds->raytrace_range_ = 5.0;
    ptr_clearing_clouds->obstacle_range_ = 6.0;
    ptr_clearing_clouds->ptr_v_cloud_->push_back({3.0, 4.0, 0.00});

    CostmapMediator::getPtrInstance()->updateData("scan", ptr_clearing_clouds);

    // back_scan
    auto ptr_clearing_clouds2 = std::make_shared<CostmapCloud>();
    ptr_clearing_clouds2->raytrace_range_ = 5.0;
    ptr_clearing_clouds2->obstacle_range_ = 5.0;
    ptr_clearing_clouds2->ptr_v_cloud_->push_back({4.0, 1.0, 0.00});

    CostmapMediator::getPtrInstance()->updateData("front_scan",
                                                  ptr_clearing_clouds2);

    // front_scan
    auto ptr_marking_clouds = std::make_shared<CostmapCloud>();
    ptr_marking_clouds->raytrace_range_ = 5.0;
    ptr_marking_clouds->obstacle_range_ = 4.0;
    ptr_marking_clouds->ptr_v_cloud_->push_back({3.0, 2.0, 0.00});
    ptr_marking_clouds->ptr_v_cloud_->push_back({0.0, 4.0, 5.0});
    ptr_marking_clouds->ptr_v_cloud_->push_back({4.0, 4.0, 0.00});
    ptr_marking_clouds->ptr_v_cloud_->push_back({-1.0, 4.0, 0.00});

    CostmapMediator::getPtrInstance()->updateData("back_scan",
                                                  ptr_marking_clouds);
    CostmapBound cb_costmap_bound = {0.00, 0.00, 2.00, 2.00};

    // world map origin, not costmap origin
    ptr_voxel_layer->updateBounds({5.0, 5.0}, cb_costmap_bound);
    printMap(*ptr_voxel_layer);
    ASSERT_EQ(8, countValues(*ptr_voxel_layer, FREE_SPACE));
    ASSERT_EQ(2, countValues(*ptr_voxel_layer, LETHAL_OBSTACLE));

    CostmapBound bound = {0.0, 0.0, 4.0, 4.0};
    ASSERT_EQ_BOUND(cb_costmap_bound, bound);

    ptr_voxel_layer->setDefaultValue(LETHAL_OBSTACLE);
    ptr_layered_costmap->addLayer(ptr_voxel_layer);
    ptr_layered_costmap->resizeMap(10, 10, 1, 0.0, 0.0);

    ASSERT_EQ(100, countValues(*ptr_voxel_layer, LETHAL_OBSTACLE));

    time_t now = time(NULL);

    sleep(7);
    time_t end = time(NULL);

    LOG(ERROR) << end - now;

    ptr_voxel_layer->updateBounds({5.0, 5.0}, cb_costmap_bound);
    printMap(*ptr_voxel_layer);

    ASSERT_EQ(2, countValues(*ptr_voxel_layer, LETHAL_OBSTACLE));
    ASSERT_EQ(98, countValues(*ptr_voxel_layer, FREE_SPACE));
  }

  void test_updateSpeedLevel() {
    auto ptr_voxel_layer = std::make_shared<VoxelLayer>();

    auto ptr_costmap_params = std::make_shared<CostmapParameters>();
    CostmapMediator::getPtrInstance()->setCostmapParameters(ptr_costmap_params);

    auto ptr_layered_costmap = std::make_shared<LayeredCostmap>(true, false);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);
    ptr_costmap_params->setParam("voxel_layer.enabled", true);

    ASSERT_EQ(ptr_voxel_layer->initialize("voxel_layer"), true);

    setArea(ptr_voxel_layer->stop_bound_, -0.8, -1.0, 1.6, 1.0);
    // clear the origin stop area
    setArea(ptr_voxel_layer->origin_stop_bound_, -0.8, -1.0, 1.6, 1.0);

    setArea(ptr_voxel_layer->slow_bound_, -1.0, -1.3, 1.8, 1.3);
    // clear the origin slow down area
    setArea(ptr_voxel_layer->origin_slow_bound_, -1.0, -1.3, 1.8, 1.3);

    auto func = [&ptr_voxel_layer](
        const WorldmapPoint &wm_point, const SpeedLevel &input_speedLevel,
        const bool &expect_result, const SpeedLevel &expect_value) {
      SpeedLevel speedLevel = input_speedLevel;
      ASSERT_EQ(ptr_voxel_layer->updateSpeedLevel(wm_point, speedLevel),
                expect_result);
      ASSERT_EQ(speedLevel, expect_value);
    };

    // left up side of stop area
    func({-0.7, 0.9}, SpeedMax, true, Stop);

    // right up side of stop area
    func({1.5, 0.9}, SpeedTwo, true, Stop);

    // left down side of slow down area
    func({-0.9, -1.1}, SpeedMax, false, SpeedTwo);

    // right down side of slow donw area
    func({0.9, -1.1}, SpeedTwo, false, SpeedTwo);

    // out of avoidance area
    func({2.0, 2.0}, SpeedMax, false, SpeedMax);
  }

  void test_getSpeedLevel() {
    auto ptr_voxel_layer = std::make_shared<VoxelLayer>();

    auto ptr_costmap_params = std::make_shared<CostmapParameters>();
    CostmapMediator::getPtrInstance()->setCostmapParameters(ptr_costmap_params);

    auto ptr_layered_costmap = std::make_shared<LayeredCostmap>(true, false);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);
    ptr_costmap_params->setParam("voxel_layer.enabled", true);
    ptr_costmap_params->setParam("voxel_layer.avoidance_cloud_sources",
                                 std::string("origin_scan"));

    ASSERT_EQ(ptr_voxel_layer->initialize("voxel_layer"), true);

    setArea(ptr_voxel_layer->stop_bound_, -0.8, -1.0, 1.6, 1.0);
    // clear the origin stop area
    setArea(ptr_voxel_layer->origin_stop_bound_, -0.8, -1.0, 1.6, 1.0);

    setArea(ptr_voxel_layer->slow_bound_, -1.0, -1.3, 1.8, 1.3);
    // clear the origin slow down area
    setArea(ptr_voxel_layer->origin_slow_bound_, -1.0, -1.3, 1.8, 1.3);

    WorldmapPoint extra_slow_down_area{0, 0};
    WorldmapPoint extra_stop_area{0, 0};

    auto func = [&ptr_voxel_layer](
        const WorldmapPoint &slow_down, const WorldmapPoint &stop,
        const SpeedLevel &expect_value, int point_num, ...) {
      auto ptr_avoidance_clouds = std::make_shared<CostmapCloud>();
      va_list args;
      va_start(args, point_num);
      while (point_num > 0) {
        ptr_avoidance_clouds->ptr_v_cloud_->push_back(
            {va_arg(args, double), va_arg(args, double), 0.0});
        point_num--;
      }
      va_end(args);

      CostmapMediator::getPtrInstance()->updateData("origin_scan",
                                                    ptr_avoidance_clouds);
      SpeedLevel speedLevel = SpeedMax;
      boost::shared_array<bool> ptr_field;  ///< 磁力场
      ptr_voxel_layer->getSpeedLevel(ptr_field, speedLevel, slow_down, stop);
      ASSERT_EQ(speedLevel, expect_value);
    };

    // test for left side of stop area
    func(extra_slow_down_area, extra_stop_area, Stop, 3, -0.7, 0.9, -0.9, 1.1,
         -2.0, 2.0);

    // test for right side of slow down area
    func(extra_slow_down_area, extra_stop_area, SpeedTwo, 2, 1.7, 1.1, -2.0,
         2.0);

    // test for right side of slow down area
    func(extra_slow_down_area, {0.15, 0.15}, Stop, 2, 1.7, 1.1, -2.0, 2.0);

    // test for out side of avoidance area
    func(extra_slow_down_area, extra_stop_area, SpeedMax, 1, -0.9, 1.5);

    func({0, 0.3}, extra_stop_area, SpeedTwo, 1, -0.9, 1.5);

    // test for rightup slow down side of slow down area
    func({0.3, 0.3}, extra_stop_area, SpeedTwo, 1, 2.0, 1.5);

    // test for rightdown side of stop area
    func({0.3, 0.3}, {1.0, -1.0}, Stop, 1, 2.0, -1.5);

    // test for left side, now slow down area and stop area resume to origin
    func(extra_slow_down_area, extra_stop_area, SpeedTwo, 1, -0.9, 1.1);

    // when the input speedLevel has been SpeedTwo/Stop, and there is no
    // voxel in current layer, the SpeedTwo will still remain the value
    // before and not be changed to SpeedMax.
    auto ptr_avoidance_clouds = std::make_shared<CostmapCloud>();
    ptr_avoidance_clouds->ptr_v_cloud_->push_back({-2.0, 2.0, 0.00});

    CostmapMediator::getPtrInstance()->updateData("origin_scan",
                                                  ptr_avoidance_clouds);

    SpeedLevel speedLevel = SpeedTwo;

    boost::shared_array<bool> ptr_field;  ///< 磁力场
    ptr_voxel_layer->getSpeedLevel(ptr_field, speedLevel, extra_slow_down_area,
                                   extra_stop_area);
    ASSERT_EQ(speedLevel, SpeedTwo);

    speedLevel = Stop;

    boost::shared_array<bool> ptr_field2;  ///< 磁力场
    ptr_voxel_layer->getSpeedLevel(ptr_field2, speedLevel, extra_slow_down_area,
                                   extra_stop_area);
    ASSERT_EQ(speedLevel, Stop);
  }
};  // class

auto ptr_voxel_layer_tester = std::make_shared<voxel_layer_tester>();

TEST(testCase, test_initialize) { ptr_voxel_layer_tester->test_initialize(); }

TEST(testCase, test_updateRaytraceBounds) {
  ptr_voxel_layer_tester->test_updateRaytraceBounds();
}

TEST(testCase, test_raytraceFreespace) {
  ptr_voxel_layer_tester->test_raytraceFreespace();
}

TEST(testCase, test_updateBounds) {
  ptr_voxel_layer_tester->test_updateBounds();
}

TEST(testCase, test_updateSpeedLevel) {
  ptr_voxel_layer_tester->test_updateSpeedLevel();
}

TEST(testCase, test_getSpeedLevel) {
  ptr_voxel_layer_tester->test_getSpeedLevel();
}
}  // namespace CVTE_BABOT
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
