/************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2019, CVTE.
* All rights reserved.
*
*@file test_layered_costmap.cpp
*
*@brief 测试layered_costmap.cpp 文件

*@author wuhuabo <wuhuabo@cvte.com>
*
*@modified wuhuabo <wuhuabo@cvte.com>
*@version current_algo.dev.1.0
*@data 2019-05-21
************************************************************************/

#include <gtest/gtest.h>
#include "costmap2d_builder.hpp"
#include "costmap2d_director.hpp"
#include "costmap_cloud.hpp"
#include "costmap_mediator.hpp"
#include "costmap_range_data.hpp"
#include "costmap_utils.hpp"
#include "layered_costmap.hpp"

namespace CVTE_BABOT {

class LayeredCostmapTester : public testing::Test {
 public:
  virtual void TestBody() {}

  void updateSpeedLevelTest() {
    auto ptr_costmap_params = std::make_shared<CostmapParameters>();
    auto ptr_costmap_mediator = CostmapMediator::getPtrInstance();
    ptr_costmap_mediator->setCostmapParameters(ptr_costmap_params);

    auto ptr_layered_costmap = std::make_shared<LayeredCostmap>(true, false);
    ptr_costmap_mediator->setLayeredCostmap(ptr_layered_costmap);

    auto builder = std::make_shared<Costmap2dBuilder>(ptr_layered_costmap);
    auto director = std::make_shared<Costmap2dDirector>(builder);
    auto layer_param = LayerParammeter();

    // set obstacle_layer's params
    ptr_costmap_params->setParam("obstacle_layer.enabled", true);
    ptr_costmap_params->setParam("obstacle_layer.avoidance_cloud_sources",
                                 std::string("origin_scan1 origin_scan2"));
    ptr_costmap_params->setParam(
        "obstacle_layer.stop_area",
        std::string("[[-0.5, 1.0], [0.5, 1.0], [0.5, -0.5], [-0.5, -0.5]]"));
    ptr_costmap_params->setParam(
        "obstacle_layer.slow_down_area",
        std::string("[[-1.0, 1.3], [1.8, 1.3], [1.8, -1.3], [-1.0, -1.3]]"));
    layer_param.addLayer(CVTE_BABOT::ObstacleLayerType, "obstacle_layer");

    // set range_sensor_layer's params
    auto names = {std::string("range_sensor_layer1"),
                  std::string("range_sensor_layer2"),
                  std::string("range_sensor_layer3")};
    int i = 1;
    for (auto name : names) {
      ptr_costmap_params->setParam(name, true);

      ptr_costmap_params->setParam(
          name + ".stop_area",
          std::string("[[-0.5, 1.0], [0.5, 1.0], [0.5, -0.5], [-0.5, -0.5]]"));
      ptr_costmap_params->setParam(
          name + ".slow_down_area",
          std::string("[[-1.0, 1.3], [1.8, 1.3], [1.8, -1.3], [-1.0, -1.3]]"));

      ptr_costmap_params->setParam(name + ".range_sensor_sources",
                                   "sonar" + std::to_string(i));
      i++;
      layer_param.addLayer(CVTE_BABOT::RangeSensorLayerType, name);
    }

    director->buildCostmap(layer_param);

    SpeedLevel speedLevel = SpeedMax;
    WorldmapPoint extra_slow_down_area{0, 0};
    WorldmapPoint extra_stop_area{0, 0};

    // test for stop area, obstacle_layer will detect obstacle
    auto ptr_avoidance_clouds = std::make_shared<CostmapCloud>();
    ptr_avoidance_clouds->s_topic_name_ = "origin_scan1";
    ptr_avoidance_clouds->ptr_v_cloud_->push_back({0.6, 0.9, 0.00});
    ptr_avoidance_clouds->ptr_v_cloud_->push_back({-0.4, 0.3, 0.00});

    auto ptr_avoidance_clouds2 = std::make_shared<CostmapCloud>();
    ptr_avoidance_clouds2->s_topic_name_ = "origin_scan2";
    ptr_avoidance_clouds2->ptr_v_cloud_->push_back({-2.0, 2.0, 0.00});

    CostmapMediator::getPtrInstance()->updateData("origin_scan1",
                                                  ptr_avoidance_clouds);
    CostmapMediator::getPtrInstance()->updateData("origin_scan2",
                                                  ptr_avoidance_clouds2);

    ptr_layered_costmap->setLayerCurrent("obstacle_layer", true);
    ASSERT_TRUE(ptr_layered_costmap->updateSpeedLevel(extra_slow_down_area,
                                                      extra_stop_area));
    WorldmapPoint repulsion;
    speedLevel = ptr_layered_costmap->getSpeedLevel(repulsion);
    ASSERT_EQ(speedLevel, Stop);

    // test for stop area, clear scan data, range_sensor_layer will detect
    // obstacle
    auto range_data = std::make_shared<CostmapRangeData>();
    range_data->s_topic_name_ = "sonar1";
    range_data->max_range_ = 5;
    range_data->min_range_ = 0;
    range_data->field_of_view_ = M_PI / 3;
    range_data->range_ = 0.3;
    range_data->sensor_pose_.d_x = 0.0;
    range_data->sensor_pose_.d_y = 0.0;
    range_data->sensor_pose_.d_yaw = 0.0;

    auto range_data2 = std::make_shared<CostmapRangeData>(*range_data);
    range_data2->s_topic_name_ = "sonar2";
    range_data2->range_ = 1.5;

    auto range_data3 = std::make_shared<CostmapRangeData>(*range_data);
    range_data3->s_topic_name_ = "sonar3";
    range_data3->range_ = 2.0;

    CostmapMediator::getPtrInstance()->updateData("sonar1", range_data);
    CostmapMediator::getPtrInstance()->updateData("sonar2", range_data2);
    CostmapMediator::getPtrInstance()->updateData("sonar3", range_data3);

    // now only range sensor data still be storaged
    ptr_avoidance_clouds->ptr_v_cloud_->clear();
    ptr_avoidance_clouds2->ptr_v_cloud_->clear();

    CostmapMediator::getPtrInstance()->updateData("origin_scan1",
                                                  ptr_avoidance_clouds);
    CostmapMediator::getPtrInstance()->updateData("origin_scan2",
                                                  ptr_avoidance_clouds2);

    ptr_layered_costmap->setLayerCurrent("obstacle_layer", true);
    ptr_layered_costmap->setLayerCurrent("range_sensor_layer1", true);
    ptr_layered_costmap->setLayerCurrent("range_sensor_layer2", true);
    ptr_layered_costmap->setLayerCurrent("range_sensor_layer3", true);
    ASSERT_TRUE(ptr_layered_costmap->updateSpeedLevel(extra_slow_down_area,
                                                      extra_stop_area));

    speedLevel = ptr_layered_costmap->getSpeedLevel(repulsion);
    ASSERT_EQ(speedLevel, Stop);

    // test for slow down area, scan data out side of slow down
    // area, range_sensor_layer will detect obstacle
    ptr_avoidance_clouds->ptr_v_cloud_->push_back({2.6, 3.9, 0.00});
    ptr_avoidance_clouds->ptr_v_cloud_->push_back({-2.4, 1.3, 0.00});

    ptr_avoidance_clouds2->ptr_v_cloud_->push_back({-2.0, 2.0, 0.00});

    CostmapMediator::getPtrInstance()->updateData("origin_scan1",
                                                  ptr_avoidance_clouds);
    CostmapMediator::getPtrInstance()->updateData("origin_scan2",
                                                  ptr_avoidance_clouds2);

    range_data->range_ = 2.5;
    range_data2->range_ = 1.3;
    range_data3->range_ = 3.0;

    CostmapMediator::getPtrInstance()->updateData("sonar1", range_data);
    CostmapMediator::getPtrInstance()->updateData("sonar2", range_data2);
    CostmapMediator::getPtrInstance()->updateData("sonar3", range_data3);

    ptr_layered_costmap->setLayerCurrent("obstacle_layer", true);
    ptr_layered_costmap->setLayerCurrent("range_sensor_layer1", true);
    ptr_layered_costmap->setLayerCurrent("range_sensor_layer2", true);
    ptr_layered_costmap->setLayerCurrent("range_sensor_layer3", true);
    LOG(INFO) << "only sonar ";
    ASSERT_TRUE(ptr_layered_costmap->updateSpeedLevel(extra_slow_down_area,
                                                      extra_stop_area));
    speedLevel = ptr_layered_costmap->getSpeedLevel(repulsion);
    ASSERT_EQ(speedLevel, SpeedTwo);

    // only range_data storaged now, and will remain the last speedLevel
    range_data->range_ = 2.0;
    CostmapMediator::getPtrInstance()->updateData("sonar1", range_data);

    ptr_layered_costmap->setLayerCurrent("range_sensor_layer1", true);
    ASSERT_TRUE(ptr_layered_costmap->updateSpeedLevel(extra_slow_down_area,
                                                      extra_stop_area));
    speedLevel = ptr_layered_costmap->getSpeedLevel(repulsion);
    ASSERT_EQ(speedLevel, SpeedTwo);

    // all out of avoidance area
    range_data->range_ = 4.5;
    range_data2->range_ = 4.5;
    range_data3->range_ = 4.5;

    CostmapMediator::getPtrInstance()->updateData("sonar1", range_data);
    CostmapMediator::getPtrInstance()->updateData("sonar2", range_data2);
    CostmapMediator::getPtrInstance()->updateData("sonar3", range_data3);

    ptr_layered_costmap->setLayerCurrent("range_sensor_layer1", true);
    ptr_layered_costmap->setLayerCurrent("range_sensor_layer2", true);
    ptr_layered_costmap->setLayerCurrent("range_sensor_layer3", true);
    ASSERT_TRUE(ptr_layered_costmap->updateSpeedLevel(extra_slow_down_area,
                                                      extra_stop_area));
    speedLevel = ptr_layered_costmap->getSpeedLevel(repulsion);
    ASSERT_EQ(speedLevel, SpeedMax);

    // test for right side of extra slow down area
    range_data->range_ = 2.0;
    CostmapMediator::getPtrInstance()->updateData("sonar1", range_data);
    ptr_layered_costmap->setLayerCurrent("range_sensor_layer1", true);

    extra_slow_down_area = {0.8, 0.0};
    ASSERT_TRUE(ptr_layered_costmap->updateSpeedLevel(extra_slow_down_area,
                                                      extra_stop_area));
    speedLevel = ptr_layered_costmap->getSpeedLevel(repulsion);
    ASSERT_EQ(speedLevel, SpeedTwo);

    // only scan data in avoidance area now
    // all out of avoidance area
    range_data->range_ = 4.5;
    range_data2->range_ = 4.5;
    range_data3->range_ = 4.5;

    CostmapMediator::getPtrInstance()->updateData("sonar1", range_data);
    CostmapMediator::getPtrInstance()->updateData("sonar2", range_data2);
    CostmapMediator::getPtrInstance()->updateData("sonar3", range_data3);

    ptr_layered_costmap->setLayerCurrent("range_sensor_layer1", true);
    ptr_layered_costmap->setLayerCurrent("range_sensor_layer2", true);
    ptr_layered_costmap->setLayerCurrent("range_sensor_layer3", true);
    ASSERT_TRUE(ptr_layered_costmap->updateSpeedLevel(extra_slow_down_area,
                                                      extra_stop_area));
    speedLevel = ptr_layered_costmap->getSpeedLevel(repulsion);
    ASSERT_EQ(speedLevel, SpeedMax);

    extra_slow_down_area = {0.9, 2.7};
    ptr_layered_costmap->setLayerCurrent("obstacle_layer", true);
    ASSERT_TRUE(ptr_layered_costmap->updateSpeedLevel(extra_slow_down_area,
                                                      extra_stop_area));
    speedLevel = ptr_layered_costmap->getSpeedLevel(repulsion);
    ASSERT_EQ(speedLevel, SpeedTwo);

    // test for extra stop area
    ptr_layered_costmap->setLayerCurrent("obstacle_layer", true);
    ASSERT_TRUE(ptr_layered_costmap->updateSpeedLevel(extra_slow_down_area,
                                                      {3.0, 3.0}));
    speedLevel = ptr_layered_costmap->getSpeedLevel(repulsion);
    ASSERT_EQ(speedLevel, Stop);
  }
};

TEST(LayeredCostmapTester, updateSpeedLevel) {
  LayeredCostmapTester layered_costmap_tester;
  layered_costmap_tester.updateSpeedLevelTest();
}

}  // namespace CVTE_BABOT

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
