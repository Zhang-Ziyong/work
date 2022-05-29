/************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2018, CVTE.
* All rights reserved.
*
*@file test_range_sensor_layer.cpp
*
*@brief 测试range_sensor_layer.cpp 文件

*@author yangfangping <yangfangping@cvte.com>
*
*@modified yangfangping <yangfangping@cvte.com>
*@modified chenmingjian <chenmingjian@cvte.com>
*@version current_algo.dev.1.0
*@data 2019-04-18
************************************************************************/

#include <gtest/gtest.h>
#include "costmap_mediator.hpp"
#include "costmap_range_data.hpp"
#include "costmap_testing_helper.hpp"
#include "range_sensor_layer.hpp"

namespace CVTE_BABOT {

class range_sensor_layer_tester : public testing::Test {
 public:
  virtual void TestBody() {}
  range_sensor_layer_tester() {}

  /**
   * onInitializeTest
   * @brief
   * 测试onInitialize函数
   * 1. 判断ptr_costmap_mediator_，是否获得唯一的单例地址
   * 2. 判断b_current_,uc_default_value_,layer_bound_的值
   *
   * */
  void onInitializeTest() {
    auto ptr_range_sensor_layer = std::make_shared<RangeSensorLayer>();
    EXPECT_FALSE(ptr_range_sensor_layer->initialize("sonar_layer"));

    auto ptr_costmap_params = std::make_shared<CostmapParameters>();
    CostmapMediator::getPtrInstance()->setCostmapParameters(ptr_costmap_params);
    EXPECT_FALSE(ptr_range_sensor_layer->initialize("sonar_layer"));

    auto ptr_layered_costmap = std::make_shared<LayeredCostmap>(true, true);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);
    EXPECT_TRUE(ptr_range_sensor_layer->initialize("sonar_layer"));

    ASSERT_EQ(ptr_range_sensor_layer->b_current_, true);
    ASSERT_EQ(ptr_range_sensor_layer->uc_default_value_,
              static_cast<unsigned char>(127));
    ASSERT_EQ(ptr_range_sensor_layer->layer_bound_.d_min_x,
              std::numeric_limits<double>::max());
    ASSERT_EQ(ptr_range_sensor_layer->layer_bound_.d_min_y,
              std::numeric_limits<double>::max());
    ASSERT_EQ(ptr_range_sensor_layer->layer_bound_.d_max_x,
              -std::numeric_limits<double>::max());
    ASSERT_EQ(ptr_range_sensor_layer->layer_bound_.d_max_y,
              -std::numeric_limits<double>::max());
  }

  void updateSensorDataTest() {
    auto ptr_range_sensor_layer = std::make_shared<RangeSensorLayer>();
    auto ptr_costmap_params = std::make_shared<CostmapParameters>();
    auto ptr_layered_costmap = std::make_shared<LayeredCostmap>(true, true);

    std::string sensor_source = "sonar sonar1 sonar2";
    ptr_costmap_params->setParam("sonar_layer.range_sensor_sources",
                                 sensor_source);

    auto ptr_costmap_mediator = CostmapMediator::getPtrInstance();
    ptr_costmap_mediator->setCostmapParameters(ptr_costmap_params);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);

    ptr_layered_costmap->resizeMap(10, 10, 1, 0, 0);

    ptr_range_sensor_layer->initialize("sonar_layer");
    ptr_layered_costmap->addLayer(ptr_range_sensor_layer);

    auto range_data = std::make_shared<CostmapRangeData>();
    range_data->max_range_ = 4;
    range_data->min_range_ = 0;
    range_data->field_of_view_ = M_PI / 2;
    range_data->range_ = 3;
    range_data->sensor_pose_.d_x = 5;
    range_data->sensor_pose_.d_y = 5;
    range_data->sensor_pose_.d_yaw = 0;

    // ptr_costmap_mediator->updateData("sonar_q", range_data);
    // EXPECT_FALSE(ptr_range_sensor_layer->updateSensorData());

    // range_data->range_ = -3;
    // ptr_costmap_mediator->updateData("sonar", range_data);
    // EXPECT_FALSE(ptr_range_sensor_layer->updateSensorData());

    // range_data->range_ = 5;
    // ptr_costmap_mediator->updateData("sonar", range_data);
    // EXPECT_FALSE(ptr_range_sensor_layer->updateSensorData());

    range_data->range_ = 3;
    ptr_costmap_mediator->updateData("sonar", range_data);
    EXPECT_TRUE(ptr_range_sensor_layer->updateSensorData());

    EXPECT_EQ(175, static_cast<int>(ptr_range_sensor_layer->getCost(5, 6)));
    EXPECT_EQ(175, static_cast<int>(ptr_range_sensor_layer->getCost(5, 4)));
    EXPECT_EQ(175, static_cast<int>(ptr_range_sensor_layer->getCost(6, 5)));
    EXPECT_EQ(131, static_cast<int>(ptr_range_sensor_layer->getCost(7, 5)));
    EXPECT_EQ(233, static_cast<int>(ptr_range_sensor_layer->getCost(8, 5)));
    EXPECT_EQ(128, static_cast<int>(ptr_range_sensor_layer->getCost(7, 4)));
    EXPECT_EQ(128, static_cast<int>(ptr_range_sensor_layer->getCost(7, 6)));

    // for (unsigned int i = 0; i < 10; ++i) {
    //   for (unsigned int j = 0; j < 10; ++j) {
    //     auto value = ptr_range_sensor_layer->getCost(i, j);
    //     std::cout << static_cast<int>(value) << " ";
    //   }
    //   std::cout << std::endl;
    // }

    EXPECT_DOUBLE_EQ(0.50036696733496033,
                     ptr_range_sensor_layer->sensorModel(3, 3, 0.1));

    EXPECT_DOUBLE_EQ(0.95,
                     ptr_range_sensor_layer->sensorModel(0.5, 2, M_PI / 6));
    EXPECT_DOUBLE_EQ(0.619551103792367,
                     ptr_range_sensor_layer->sensorModel(3, 0.4, 0.01));
    EXPECT_DOUBLE_EQ(0.51740416599508321,
                     ptr_range_sensor_layer->sensorModel(3, 2, 0.01));
    EXPECT_DOUBLE_EQ(0.50000607635215011,
                     ptr_range_sensor_layer->sensorModel(3, 4, 0.01));
  }

  void updateBoundsTest() {
    auto ptr_range_sensor_layer = std::make_shared<RangeSensorLayer>();
    auto ptr_costmap_params = std::make_shared<CostmapParameters>();
    auto ptr_layered_costmap = std::make_shared<LayeredCostmap>(true, true);

    std::string sensor_source = "sonar";
    ptr_costmap_params->setParam("sonar_layer.range_sensor_sources",
                                 sensor_source);

    auto ptr_costmap_mediator = CostmapMediator::getPtrInstance();
    ptr_costmap_mediator->setCostmapParameters(ptr_costmap_params);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);

    ptr_layered_costmap->resizeMap(10, 10, 1, 0, 0);

    ptr_range_sensor_layer->initialize("sonar_layer");
    ptr_layered_costmap->addLayer(ptr_range_sensor_layer);

    auto range_data = std::make_shared<CostmapRangeData>();
    range_data->max_range_ = 4;
    range_data->min_range_ = 0;
    range_data->field_of_view_ = M_PI / 2;
    range_data->range_ = 3;
    range_data->sensor_pose_.d_x = 5;
    range_data->sensor_pose_.d_y = 5;
    range_data->sensor_pose_.d_yaw = 0;
    ptr_costmap_mediator->updateData("sonar", range_data);

    std::vector<WorldmapPoint> polygon;
    polygon.push_back({-1.0, -1.0});
    polygon.push_back({1.0, -1.0});
    polygon.push_back({1.0, 1.0});
    polygon.push_back({-1.0, 1.0});
    ptr_layered_costmap->setFootprint(polygon);

    WorldmapPose wp_robot_pose = {3, 3, 0};
    CostmapBound cb_costmap_bound = {1e30, 1e30, -1e30, -1e30};

    ptr_range_sensor_layer->updateBounds(wp_robot_pose, cb_costmap_bound);

    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_min_x, 5);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_min_y, 2.454415587728429);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_max_x, 8);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_max_y, 7.5455844122715714);

    cb_costmap_bound = {1, 2, 7, 8};

    ptr_range_sensor_layer->updateBounds(wp_robot_pose, cb_costmap_bound);

    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_min_x, 1);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_min_y, 2);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_max_x, 7);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_max_y, 8);

    cb_costmap_bound = {0, 0, 1, 1};

    ptr_range_sensor_layer->updateBounds(wp_robot_pose, cb_costmap_bound);

    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_min_x, 0);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_min_y, 0);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_max_x, 1);
    ASSERT_DOUBLE_EQ(cb_costmap_bound.d_max_y, 1);
  }

  void updateCostTest() {
    auto ptr_range_sensor_layer = std::make_shared<RangeSensorLayer>();
    auto ptr_costmap_params = std::make_shared<CostmapParameters>();
    auto ptr_layered_costmap = std::make_shared<LayeredCostmap>(true, false);

    std::string sensor_source = "sonar";
    ptr_costmap_params->setParam("sonar_layer.range_sensor_sources",
                                 sensor_source);

    auto ptr_costmap_mediator = CostmapMediator::getPtrInstance();
    ptr_costmap_mediator->setCostmapParameters(ptr_costmap_params);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);

    ptr_layered_costmap->resizeMap(10, 10, 1, 0, 0);

    ptr_range_sensor_layer->initialize("sonar_layer");
    ptr_layered_costmap->addLayer(ptr_range_sensor_layer);

    auto range_data = std::make_shared<CostmapRangeData>();
    range_data->max_range_ = 4;
    range_data->min_range_ = 0;
    range_data->field_of_view_ = M_PI / 2;
    range_data->range_ = 3;
    range_data->sensor_pose_.d_x = 5;
    range_data->sensor_pose_.d_y = 5;
    range_data->sensor_pose_.d_yaw = 0;
    ptr_costmap_mediator->updateData("sonar", range_data);

    std::vector<WorldmapPoint> polygon;
    polygon.push_back({-1.0, -1.0});
    polygon.push_back({1.0, -1.0});
    polygon.push_back({1.0, 1.0});
    polygon.push_back({-1.0, 1.0});
    ptr_layered_costmap->setFootprint(polygon);

    WorldmapPose wp_robot_pose = {5, 5, 0};
    CostmapBound cb_costmap_bound = {1e30, 1e30, -1e30, -1e30};

    ptr_range_sensor_layer->updateBounds(wp_robot_pose, cb_costmap_bound);

    auto master_costmap =
        std::make_shared<Costmap2d>(*ptr_layered_costmap->getCostmap());
    EXPECT_TRUE(
        ptr_range_sensor_layer->updateCosts(master_costmap, 0, 0, 10, 10));

    EXPECT_EQ(254, static_cast<int>(master_costmap->getCost(8, 5)));
  }

  void handleRangeDataTest() {
    auto ptr_range_sensor_layer = std::make_shared<RangeSensorLayer>();

    auto ptr_costmap_params = std::make_shared<CostmapParameters>();
    CostmapMediator::getPtrInstance()->setCostmapParameters(ptr_costmap_params);

    auto ptr_layered_costmap = std::make_shared<LayeredCostmap>(true, false);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);
    ptr_costmap_params->setParam("range_sensor_layer.enabled", true);

    ASSERT_EQ(ptr_range_sensor_layer->initialize("range_sensor_layer"), true);

    setArea(ptr_range_sensor_layer->stop_bound_, -2.0, -2.0, 2.0, 2.0);
    // clear the origin stop area
    setArea(ptr_range_sensor_layer->slow_bound_, -4.0, -4.0, 4.0, 4.0);

    auto range_data = std::make_shared<CostmapRangeData>();

    range_data->max_range_ = 8;
    range_data->min_range_ = 0;
    range_data->sensor_pose_.d_x = 0;
    range_data->sensor_pose_.d_y = 0;
    range_data->field_of_view_ = M_PI / 3;
    range_data->sensor_pose_.d_yaw = 0;

    auto func = [&ptr_range_sensor_layer](
        const std::shared_ptr<CostmapRangeData> &ptr, const bool &expect_result,
        const SpeedLevel &expect_value) {
      SpeedLevel speedLevel = SpeedMax;
      std::vector<unsigned int> v_index;  ///< 磁力场
      ASSERT_EQ(
          ptr_range_sensor_layer->handleRangeData(v_index, ptr, speedLevel),
          expect_result);
      ASSERT_EQ(speedLevel, expect_value);
    };

    // test for right side of slow down area
    range_data->range_ = 3.0;
    func(range_data, false, SpeedTwo);

    // test for right side of stop area
    range_data->range_ = 1;
    func(range_data, true, Stop);

    // test for out side of avoidance area
    range_data->range_ = 5;
    func(range_data, false, SpeedMax);

    // the calculated obstacle point in rightdown side of stop area
    range_data->range_ = 2.1 / 1.2;
    range_data->sensor_pose_.d_yaw = M_PI / 6;
    func(range_data, true, Stop);

    // the calculated obstacle point in rightup side of stop area
    range_data->range_ = 2.1 / 1.2;
    range_data->sensor_pose_.d_yaw = -M_PI / 6;
    func(range_data, true, Stop);

    // the calculated obstacle point in rightup side of slow down area
    range_data->range_ = 4.1 / 1.2;
    range_data->sensor_pose_.d_yaw = -M_PI / 6;
    func(range_data, false, SpeedTwo);

    // the calculated obstacle point in side of stop area
    range_data->range_ = 2.1 / 1.2;
    range_data->sensor_pose_.d_yaw = 0.0;
    func(range_data, true, Stop);

    // the calculated obstacle point out side of stop area and in side of slow
    // down area
    range_data->range_ = 2.1 / 1.2;
    range_data->sensor_pose_.d_yaw = M_PI / 4;
    func(range_data, false, SpeedTwo);
  }

  void getSpeedLevelTest() {
    auto ptr_range_sensor_layer = std::make_shared<RangeSensorLayer>();

    auto ptr_costmap_params = std::make_shared<CostmapParameters>();
    CostmapMediator::getPtrInstance()->setCostmapParameters(ptr_costmap_params);

    auto ptr_layered_costmap = std::make_shared<LayeredCostmap>(true, false);
    CostmapMediator::getPtrInstance()->setLayeredCostmap(ptr_layered_costmap);
    ptr_costmap_params->setParam("range_sensor_layer.enabled", true);

    std::string sensor_source = "sonar sonar1 sonar2";
    ptr_costmap_params->setParam("range_sensor_layer.range_sensor_sources",
                                 sensor_source);

    ASSERT_EQ(ptr_range_sensor_layer->initialize("range_sensor_layer"), true);

    setArea(ptr_range_sensor_layer->stop_bound_, -0.5, -0.5, 1.0, 0.5);
    // clear the origin stop area
    setArea(ptr_range_sensor_layer->origin_stop_bound_, -0.5, -0.5, 1.0, 0.5);

    setArea(ptr_range_sensor_layer->slow_bound_, -1.0, -1.3, 1.8, 1.3);
    // clear the origin slow down area
    setArea(ptr_range_sensor_layer->origin_slow_bound_, -1.0, -1.3, 1.8, 1.3);

    auto func = [&ptr_range_sensor_layer](
        const std::string &topic, const std::shared_ptr<CostmapRangeData> &ptr,
        const WorldmapPoint &slow_down, const WorldmapPoint &stop,
        const SpeedLevel &expect_value, const int &log_index) {
      CostmapMediator::getPtrInstance()->updateData(topic, ptr);
      SpeedLevel speedLevel = SpeedMax;
      boost::shared_array<bool> ptr_field = makeFiled();  ///< 磁力场
      resetFiled(ptr_field);
      ptr_range_sensor_layer->getSpeedLevel(ptr_field, speedLevel, slow_down,
                                            stop);
      LOG(ERROR) << "test index: " << log_index;
      ASSERT_EQ(speedLevel, expect_value);
    };

    WorldmapPoint extra_slow_down_area;
    WorldmapPoint extra_stop_area;

    // test for down side of slow down area
    auto range_data = std::make_shared<CostmapRangeData>();
    range_data->max_range_ = 4;
    range_data->min_range_ = 0;
    range_data->field_of_view_ = M_PI / 3;
    range_data->range_ = 0.9 / 1.2;
    range_data->sensor_pose_.d_x = 0.0;
    range_data->sensor_pose_.d_y = 0.3;
    range_data->sensor_pose_.d_yaw = M_PI / 2;

    // test up side
    func("sonar", range_data, extra_slow_down_area, extra_stop_area, SpeedTwo,
         1);

    // test for right side of slow down area
    range_data->range_ = 1.5 / 1.2;
    range_data->sensor_pose_.d_x = 0.0;
    range_data->sensor_pose_.d_y = 0.0;
    range_data->sensor_pose_.d_yaw = 0.0;

    func("sonar", range_data, extra_slow_down_area, extra_stop_area, SpeedTwo,
         2);

    // test left side
    range_data->range_ = 1.5 / 1.2;
    range_data->sensor_pose_.d_yaw = -M_PI;

    // data has been clear after updating so there is no data, but the
    // speedLevel remain the last until new topic comes.
    func("", range_data, extra_slow_down_area, extra_stop_area, SpeedTwo, 3);

    // test down side
    range_data->range_ = 1.6 / 1.2;
    range_data->sensor_pose_.d_yaw = -M_PI / 2;
    func("sonar", range_data, extra_slow_down_area, extra_stop_area, SpeedMax,
         4);

    // add extra avoidance area in down side
    extra_slow_down_area = {0.0, -1.0};
    func("sonar", range_data, extra_slow_down_area, extra_stop_area, SpeedTwo,
         5);

    // test for rightdown of extra stop area
    func("sonar", range_data, extra_slow_down_area, {1.0, -1.0}, Stop, 6);

    // test for multi layer
    SpeedLevel speedLevel = SpeedMax;
    extra_slow_down_area = {0.0, 0.0};
    range_data->range_ = 1.8 / 1.2;
    CostmapMediator::getPtrInstance()->updateData("sonar", range_data);

    auto range_data1 = std::make_shared<CostmapRangeData>(*range_data);
    range_data1->range_ = 1.0 / 1.2;
    CostmapMediator::getPtrInstance()->updateData("sonar1", range_data1);

    auto range_data2 = std::make_shared<CostmapRangeData>(*range_data);
    range_data2->range_ = 0.3 / 1.2;
    CostmapMediator::getPtrInstance()->updateData("sonar2", range_data2);
    boost::shared_array<bool> ptr_field = makeFiled();  ///< 磁力场
    resetFiled(ptr_field);

    ptr_range_sensor_layer->getSpeedLevel(
        ptr_field, speedLevel, extra_slow_down_area, extra_stop_area);
    ASSERT_EQ(speedLevel, Stop);

    speedLevel = SpeedMax;
    CostmapMediator::getPtrInstance()->updateData("sonar", range_data);
    CostmapMediator::getPtrInstance()->updateData("sonar1", range_data1);
    range_data2->range_ = 4.0 / 1.2;
    CostmapMediator::getPtrInstance()->updateData("sonar2", range_data2);

    ptr_range_sensor_layer->getSpeedLevel(
        ptr_field, speedLevel, extra_slow_down_area, extra_stop_area);
    ASSERT_EQ(speedLevel, SpeedTwo);

    range_data->range_ = 4.0 / 1.2;
    CostmapMediator::getPtrInstance()->updateData("sonar", range_data);
    range_data1->range_ = 4.0 / 1.2;
    CostmapMediator::getPtrInstance()->updateData("sonar1", range_data1);

    func("sonar", range_data, extra_slow_down_area, extra_stop_area, SpeedMax,
         7);
  }
};

TEST(range_sensor_layer_tester, onInitializeTest) {
  range_sensor_layer_tester range_tester;
  range_tester.onInitializeTest();
}

// TEST(range_sensor_layer_tester, updateSensorDataTest) {
//   range_sensor_layer_tester range_tester;
//   range_tester.updateSensorDataTest();
// }

// TEST(range_sensor_layer_tester, sensorModelTest) {
//   range_sensor_layer_tester range_tester;
//   range_tester.updateBoundsTest();
// }

// TEST(range_sensor_layer_tester, updateCostTest) {
//   range_sensor_layer_tester range_tester;
//   range_tester.updateCostTest();
// }

TEST(range_sensor_layer_tester, handleRangeDataTest) {
  range_sensor_layer_tester range_tester;
  range_tester.handleRangeDataTest();
}

TEST(range_sensor_layer_tester, getSpeedLevelTest) {
  range_sensor_layer_tester range_tester;
  range_tester.getSpeedLevelTest();
}

}  // namespace CVTE_BABOT
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
