/*
* brief : map_grid_cost_function.cpp 文件的单元测试
* create: 2019-04-15
* tester: caoyong
*/

#ifndef TEST_MAP_GRID_COSTMAP_FUNCTION_HPP
#define TEST_MAP_GRID_COSTMAP_FUNCTION_HPP

#include <gtest/gtest.h>
#include <memory>
#include "costmap2d_builder.hpp"
#include "costmap2d_director.hpp"
#include "costmap_mediator.hpp"
#include "costmap_model.hpp"
#include "costmap_testing_helper.hpp"
#include "map_grid_cost_function.hpp"
#include "trajectory.hpp"
#include "world_map_data.hpp"

namespace CVTE_BABOT {

class MapGridCostFunctionTest : public testing::Test {
 public:
  virtual void TestBody() {}

  MapGridCostFunctionTest() {
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

    auto builder = std::make_shared<Costmap2dBuilder>(ptr_layered_costmap_);
    auto director = std::make_shared<Costmap2dDirector>(builder);
    director->buildCostmap(layer_param_);

    ptr_layered_costmap_->updateMap(robot_current_pose_);

    ptr_costmap_ =
        std::make_shared<Costmap2d>(*(ptr_costmap_mediator_->getCostmap()));
  }

  /**
* @brief 测试说明
* １．测试构造函数和复制
* */

  void construct_test() {
    CostmapModel::getPtrInstance()->setPtrCostmap(ptr_costmap_);
    ptr_costmap_->setCost(5, 5, LETHAL_OBSTACLE);
    ptr_costmap_->setCost(5, 6, LETHAL_OBSTACLE);
    ptr_costmap_->setCost(5, 7, LETHAL_OBSTACLE);
    printMap(*(ptr_costmap_));

    auto map_grid_ptr = std::make_shared<MapGridCostFunction>(0.0, 0.0, false);
    auto map_grid = map_grid_ptr;

    EXPECT_NE(nullptr, map_grid_ptr.get());
    EXPECT_NE(nullptr, map_grid.get());

    for (int i = 0; i < 10; ++i) {
      for (int j = 0; j < 10; ++j) {
        EXPECT_EQ(map_grid_ptr->getCellCosts(i, j),
                  map_grid->getCellCosts(i, j));
      }
    }
  }

  /**
* @brief 测试说明
* １．测试路径的得分函数
* */
  void map_grid_socre_test() {
    typedef struct {
      double x;    // distance between two wheel
      double y;    // v of base
      double yaw;  // w of base
    } test_params;

    test_params test_table[] = {
        // x ,y ,yaw
        {1.0, 1.18339, 0},  {1.1, 1.18758, 0},  {1.2, 1.19102, 0},
        {1.3, 1.19496, 0},  {1.4, 1.19957, 0},  {1.5, 1.20383, 0},
        {1.62, 1.21052, 0}, {1.61, 1.21488, 0}, {1.62, 1.22229, 0},
        {1.65, 1.22691, 0}, {1.6, 1.23177, 0},  {1.69, 1.237, 0},
        {1.6, 1.24246, 0},  {1.63, 1.25199, 0}};
    std::vector<Pose2d> target_poses;
    for (auto value : test_table) {
      Pose2d pose;
      pose.setPose(value.x, value.y, value.yaw);
      target_poses.push_back(pose);
    }

    ptr_costmap_->setCost(1, 2, LETHAL_OBSTACLE);
    ptr_costmap_->setCost(2, 3, LETHAL_OBSTACLE);
    ptr_costmap_->setCost(3, 4, LETHAL_OBSTACLE);

    auto traject = std::make_shared<Trajectory>();

    traject->addPoint(1.0, 1.0, 0.0);
    traject->addPoint(2.0, 2.0, 0.0);
    traject->addPoint(3.0, 3.0, 0.0);
    traject->addPoint(4.0, 4.0, 0.0);
    traject->addPoint(5.0, 5.0, 0.0);
    traject->addPoint(30.0, 30.0, 0.0);
    CostmapModel::getPtrInstance()->setPtrCostmap(ptr_costmap_);
    auto map_grid_ptr =
        std::make_shared<MapGridCostFunction>(10.0, 10.0, false);
    map_grid_ptr->setTargetPoses(target_poses);
    map_grid_ptr->prepare();
    EXPECT_NE(0, map_grid_ptr->scoreTrajectory(*traject));

    ptr_costmap_->setCost(3, 3, LETHAL_OBSTACLE);
    map_grid_ptr = std::make_shared<MapGridCostFunction>(0.0, 0.0, false);
    map_grid_ptr->setTargetPoses(target_poses);
    map_grid_ptr->prepare();
    EXPECT_EQ(-3.0, map_grid_ptr->scoreTrajectory(*traject));

    traject->addPoint(-1.0, -1.0, 0.0);
    traject->addPoint(1.0, -3.0, 0.0);
    traject->addPoint(-5.0, 1.0, 0.0);
    traject->addPoint(0.0, 0.0, 0.0);
    traject->addPoint(30.0, -30.0, 0.0);
    EXPECT_EQ(-3.0, map_grid_ptr->scoreTrajectory(*traject));
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

TEST(MapGridCostTest, construct_test) {
  MapGridCostFunctionTest map_cost_test;
  map_cost_test.construct_test();
}

TEST(MapGridCostTest, map_grid_score_test) {
  MapGridCostFunctionTest map_cost_test;
  map_cost_test.map_grid_socre_test();
}

}  // namespace CVTE_BABOT

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

#endif