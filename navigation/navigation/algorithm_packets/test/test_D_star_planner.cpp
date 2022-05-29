/*
 * brief : D星算法 的整体测试
 *
 * create: 2019-04-12
 *
 * tester: liangjiajun
 */

#ifndef TEST_D_STAR_PLANNER_TEST
#define TEST_D_STAR_PLANNER_TEST

#include <layered_costmap.hpp>
#include "costmap2d_builder.hpp"
#include "costmap2d_director.hpp"
#include "costmap_mediator.hpp"
#include "costmap_testing_helper.hpp"
#include "world_map_data.hpp"

#include "dijkstra/D_star.hpp"
#include "dijkstra/expander.hpp"
#include "dijkstra/potential_calculator.hpp"

#include <pose2d/pose2d.hpp>

#include <gtest/gtest.h>
#include <iostream>
#include <string>

namespace CVTE_BABOT {

class DstarPlannerTest : public testing::Test {
 public:
  virtual void TestBody() {}
  DstarPlannerTest();

  void testCostMap();
  void tesetPotentialCalculator();
  void testOutLineMap();
  void testGetCost();
  void testPotentialCalc();
  void testPotentials();
  void testGetPath();
  void printPotential(const boost::shared_array<float> &ptr_potential_array,
                      int x, int y);
  void printCostMap(const boost::shared_array<unsigned char> &costmap,
                    int size_x, int size_y);

 private:
  unsigned int map_x_size_ = 0;
  unsigned int map_y_size_ = 0;
  std::shared_ptr<LayeredCostmap> ptr_layers_ = nullptr;
  // std::shared_ptr<Costmap2DROS> ptr_costmap_2d_ = nullptr;
  std::shared_ptr<DijkstraExpansion> ptr_dijkstar_ = nullptr;
  // std::shared_ptr<rclcpp::Node> ptr_node_ = nullptr;
  std::shared_ptr<CostmapMediator> ptr_costmap_mediator_;
  std::shared_ptr<CostmapParameters> ptr_costmap_params_;
  LayerParammeter layer_param_;
  std::shared_ptr<LayeredCostmap> ptr_layered_costmap_;
  std::shared_ptr<Costmap2d> ptr_costmap_;
  WorldmapPose robot_current_pose_;
};

DstarPlannerTest::DstarPlannerTest() {
  ptr_costmap_mediator_ = CostmapMediator::getPtrInstance();
  std::shared_ptr<WorldmapData> ptr_wm_data;

  std::string layer_name("static_layer");
  std::string inf_layer_name("inflation_layer");
  bool track_unknown_space = true;
  bool use_maximum = false;
  bool trinary_costmap = true;
  int lethal_cost_threshold = 100;
  int unknown_cost_value = 255;
  double inflation_radius = 0.6;
  double cost_scaling_factor = 3.0;

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

  ptr_costmap_mediator_->setCostmapParameters(ptr_costmap_params_);
  ptr_costmap_mediator_->registerUpdateFunc<std::shared_ptr<WorldmapData>>(
      "static_map", ptr_wm_data);

  auto ptr_mapdata_ = std::make_shared<WorldmapData>();
  bool read_map = ptr_mapdata_->readMapFromYaml(
      "./src/navigation/navigation/algorithm_packets/test/edit_map.yaml");
  std::cout << "Read map result: " << read_map << std::endl;
  ptr_costmap_mediator_->updateData("static_map", ptr_mapdata_);
  // ptr_costmap_mediator_->setCostmapParameters(ptr_costmap_params_);
  ptr_costmap_mediator_->setResetStaticMapFlag(true);
  ptr_layered_costmap_ = std::make_shared<LayeredCostmap>(false, false);

  double map_width_meters = 30.0, map_height_meters = 30.0, origin_x = 0.0,
         origin_y = 0.0;
  if (!ptr_layered_costmap_->isSizeLocked()) {
    ptr_layered_costmap_->resizeMap((unsigned int) (map_width_meters / 1.0),
                                    (unsigned int) (map_height_meters / 1.0),
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
  bool update_layer = ptr_layered_costmap_->updateMap(robot_current_pose_);
  std::cout << "update_layer " << update_layer << std::endl;
  ptr_costmap_mediator_->getData("static_costmap", ptr_costmap_);

  // ptr_costmap_ =
  //     std::make_shared<Costmap2d>(*(ptr_costmap_mediator_->getCostmap()));
  // ptr_costmap_2d_ = std::make_shared<Costmap2DROS>(
  //     ptr_node, std::string("dijkstar_tester"));
  // ptr_layers_ = ptr_costmap_2d_->getLayeredCostmap();
}

void DstarPlannerTest::testCostMap() {
  // ptr_layers_->updateMap(0, 0, 0.0);
  map_x_size_ = ptr_costmap_->getSizeInCellsX();
  map_y_size_ = ptr_costmap_->getSizeInCellsY();
  // printMap(*(ptr_costmap_));
  std::cout << map_x_size_ << " , " << map_y_size_ << std::endl;
}

void DstarPlannerTest::printPotential(
    const boost::shared_array<float> &ptr_potential_array, int x, int y) {
  if (ptr_potential_array.get()) {
    printf("Potential Map :\n");
    for (int i = 0; i < y; i++) {
      for (int j = 0; j < x; j++) {
        float tmp = ptr_potential_array[i * x + j];
        if (tmp > 500000) {
          printf("%5.0d", -1);
        } else {
          printf("%5.0f", tmp * 10);
        }
      }
      printf("\n\n");
    }
  }
}

void DstarPlannerTest::printCostMap(
    const boost::shared_array<unsigned char> &costmap, int size_x, int size_y) {
  for (int i = 0; i < size_y; ++i) {
    for (int j = 0; j < size_x; ++j) {
      printf("%4d", (int) costmap[i * size_x + j]);
    }
    std::cout << std::endl << std::endl;
  }
}

void DstarPlannerTest::tesetPotentialCalculator() {
  ptr_dijkstar_ = std::make_shared<DijkstraExpansion>(map_x_size_, map_y_size_);
  auto costCharmap = ptr_costmap_->getCharMap();
  // printCostMap(costCharmap, map_x_size_, map_y_size_);
  ptr_dijkstar_->outlineMap(costCharmap, map_x_size_, map_y_size_,
                            LETHAL_OBSTACLE);

  // ptr_dijkstar_->setNeutralCost((unsigned char)50);
  if (ptr_dijkstar_->calculatePotentials(costCharmap, 3.0, 27.0, 25.0, 25.0,
                                         map_x_size_ * map_y_size_ * 2)) {
    std::cout << "Calc succeed!" << std::endl;
    // printPotential(ptr_dijkstar_->getPotentialMap(), map_x_size_,
    // map_y_size_);
  } else {
    std::cout << "calc potential failed ." << std::endl;
  }
  std::vector<Pose2d> path;
  if (ptr_dijkstar_->getPath(3.0, 27.0, 28.0, 28.0, path)) {
    std::cout << "Path size = " << path.size() << std::endl;
    for (unsigned int i = 0; i < path.size(); ++i) {
      std::cout << "(" << path[i].getX() << ", " << path[i].getY() << ")"
                << std::endl;
      costCharmap[(int) (path[i].getY() + 0.5) * map_x_size_ +
                  (int) (path[i].getX() + 0.5)] = 1;
    }
    // printCostMap(costCharmap, map_x_size_, map_y_size_);
  } else {
    std::cout << "Can't find a path." << std::endl;
  }
  std::vector<Pose2d> D_path;
  Pose2d start, end;
  start.x = 3.0;
  start.y = 27.0;
  end.x = 28.0;
  end.y = 28.0;
  if (ptr_dijkstar_->makePlan(costCharmap, start, end, D_path)) {
    std::cout << "D_path: " << std::endl;
    for (unsigned int i = 0; i < D_path.size(); ++i) {
      std::cout << "(" << D_path[i].getX() << ", " << D_path[i].getY() << ")"
                << std::endl;
      costCharmap[(int) (D_path[i].getY() + 0.5) * map_x_size_ +
                  (int) (D_path[i].getX() + 0.5)] = 1;
    }
  }
}

// /**
//  * @brief 测试说明
//  * １．测试将costmap的边界的限制，避免搜索路径时越界搜索
//  * */

void DstarPlannerTest::testOutLineMap() {
  ptr_dijkstar_.reset();
  ptr_dijkstar_ = std::make_shared<DijkstraExpansion>(map_x_size_, map_y_size_);
  boost::shared_array<unsigned char> costmap = nullptr;
  auto costCharMap = ptr_costmap_->getCharMap();
  // 传入空指针时不能计算边界且不异常
  EXPECT_EQ(false, ptr_dijkstar_->outlineMap(costmap, map_x_size_, map_y_size_,
                                             LETHAL_OBSTACLE));
  costmap = boost::shared_array<unsigned char>(new unsigned char[100]);
  EXPECT_EQ(false,
            ptr_dijkstar_->outlineMap(costmap, map_x_size_ + 10,
                                      map_y_size_ - 10, LETHAL_OBSTACLE));

  // 传入正常的costmap指针，将边界置为致命值
  EXPECT_EQ(true, ptr_dijkstar_->outlineMap(costCharMap, map_x_size_,
                                            map_y_size_, LETHAL_OBSTACLE));

  // 检查四条边的代价值是否为致命
  auto outlineMap = costCharMap.get();
  for (unsigned int i = 0; i < map_x_size_; ++i) {
    EXPECT_EQ(LETHAL_OBSTACLE, *outlineMap++);
  }
  outlineMap = costCharMap.get() + (map_y_size_ - 1) * map_x_size_;
  for (unsigned int i = 0; i < map_x_size_; ++i) {
    EXPECT_EQ(LETHAL_OBSTACLE, *outlineMap++);
  }
  outlineMap = costCharMap.get();
  for (unsigned int i = 0; i < map_y_size_; ++i, outlineMap += map_x_size_) {
    EXPECT_EQ(LETHAL_OBSTACLE, *outlineMap);
  }
  outlineMap = costCharMap.get() + map_x_size_ - 1;
  for (unsigned int i = 0; i < map_y_size_; ++i, outlineMap += map_x_size_) {
    EXPECT_EQ(LETHAL_OBSTACLE, *outlineMap);
  }
}

/**
 * @brief 测试说明
 * １．测试获取地图的cost值，包含越界访问，越界时返回致命值
 * */

void DstarPlannerTest::testGetCost() {
  ptr_dijkstar_.reset();
  ptr_dijkstar_ = std::make_shared<DijkstraExpansion>(map_x_size_, map_y_size_);
  auto costCharMap = ptr_costmap_->getCharMap();
  float cost_value = 0.0;
  bool cost_range = false;
  // 遍历时 +10 是为了测试越界访问， 获得的代价值都是在[0, 254]范围内
  for (unsigned int i = 0; i < map_x_size_ + 10; ++i) {
    for (unsigned int j = 0; j < map_y_size_ + 10; ++j) {
      cost_value = ptr_dijkstar_->getCost(costCharMap, i * map_x_size_ + j);
      cost_range = (cost_value > 0.0) && (cost_value < 254);
      EXPECT_EQ(true, cost_range);
    }
  }
}

/**
 * @brief 测试说明
 * １．测试在不同代价值下，对于某一点计算的势场值
 * */

void DstarPlannerTest::testPotentialCalc() {
  ptr_dijkstar_.reset();
  ptr_dijkstar_ = std::make_shared<DijkstraExpansion>(map_x_size_, map_y_size_);
  auto potentialMap = ptr_dijkstar_->getPotentialMap();
  potentialMap =
      boost::shared_array<float>(new float[map_x_size_ * map_y_size_]);
  float potential_value = 0.0;
  for (int i = 1; i < 254; ++i) {
    potential_value = ptr_dijkstar_->p_calc_->calculatePotential(
        potentialMap, (unsigned char) i, 50);
    // std::cout << "potential_value = " << potential_value << std::endl;
    EXPECT_NE(0.0, potential_value);
  }
}

/**
 * @brief 测试说明
 * １．测试在不同的起点和终点时，D×关于势场的计算结果
 * */

void DstarPlannerTest::testPotentials() {
  ptr_dijkstar_.reset();
  ptr_dijkstar_ = std::make_shared<DijkstraExpansion>(map_x_size_, map_y_size_);
  auto costCharMap = ptr_costmap_->getCharMap();
  ptr_dijkstar_->outlineMap(costCharMap, map_x_size_, map_y_size_,
                            LETHAL_OBSTACLE);

  typedef struct {
    double start_x;
    double start_y;
    double target_x;
    double target_y;
  } test_params;

  test_params test_true[] = {{2.0, 3.0, 18.0, 19.0}, {12.0, 3.0, 8.0, 19.0},
                             {2.0, 13.0, 2.0, 19.0}, {3.0, 2.0, 26.0, 19.0},
                             {2.0, 8.0, 18.0, 15.0}, {4.0, 6.0, 18.0, 15.0},
                             {8.0, 3.0, 18.0, 19.0}};

  for (auto pose : test_true) {
    bool potentials_result = ptr_dijkstar_->calculatePotentials(
        costCharMap, pose.start_x, pose.start_y, pose.target_x, pose.target_y,
        map_x_size_ * map_y_size_ * 2);
    std::cout << "potentials_result = " << potentials_result << std::endl;
    EXPECT_EQ(true, potentials_result);
  }

  test_params test_false[] = {
      {2.0, -3.0, -18.0, -19.0}, {-12.0, -3.0, 8.0, -19.0},
      {2.0, -13.0, -2.0, -19.0}, {3.0, -2.0, -26.0, -19.0},
      {2.0, -8.0, -18.0, 15.0},  {-4.0, -6.0, -18.0, 15.0},
      {8.0, -3.0, 18.0, -19.0}};

  for (auto pose : test_false) {
    bool potentials_result = ptr_dijkstar_->calculatePotentials(
        costCharMap, pose.start_x, pose.start_y, pose.target_x, pose.target_y,
        map_x_size_ * map_y_size_ * 2);
    // std::cout << "potentials_result = " << potentials_result << std::endl;
    EXPECT_EQ(false, potentials_result);
  }
}

/**
 * @brief 测试说明
 * １．测试在不同的起点和终点中，在计算势场后，获取势场中的路径结果
 * */

void DstarPlannerTest::testGetPath() {
  if (ptr_costmap_ == nullptr) {
    std::cout << "ptr_costmap is nullptr" << std::endl;
    return;
  } else {
    std::cout << "ptr_costmap is not nullptr" << std::endl;
  }
  map_x_size_ = ptr_costmap_->getSizeInCellsX();
  map_y_size_ = ptr_costmap_->getSizeInCellsY();
  std::cout << "map size: " << map_x_size_ << " " << map_y_size_ << std::endl;
  ptr_dijkstar_.reset();
  std::cout << "reset ptr_dijkstra succeed" << std::endl;
  ptr_dijkstar_ = std::make_shared<DijkstraExpansion>(map_x_size_, map_y_size_);
  auto costCharMap = ptr_costmap_->getCharMap();
  ptr_dijkstar_->outlineMap(costCharMap, map_x_size_, map_y_size_,
                            LETHAL_OBSTACLE);

  typedef struct {
    double start_x;
    double start_y;
    double target_x;
    double target_y;
  } test_params;

  // test_params test_true[] = {{2.0, 3.0, 18.0, 19.0}, {12.0, 3.0, 8.0, 19.0},
  //                            {2.0, 13.0, 2.0, 19.0}, {3.0, 2.0, 26.0, 19.0},
  //                            {2.0, 8.0, 18.0, 15.0}, {4.0, 6.0, 18.0, 15.0},
  //                            {8.0, 3.0, 18.0, 19.0}};
  test_params test_true[] = {{24.379, 5.678, -2.28, 5.11}};
  CostmapPoint start_point;
  CostmapPoint end_point;
  ptr_costmap_->worldToMap(WorldmapPoint(24.379, 5.678), start_point);
  ptr_costmap_->worldToMap(WorldmapPoint(-2.28, 5.11), end_point);
  // for (auto pose : test_true) {
  ptr_dijkstar_->calculatePotentials(
      costCharMap, start_point.ui_x, start_point.ui_y, end_point.ui_x,
      end_point.ui_y, map_x_size_ * map_y_size_ * 2);
  // printPotential(ptr_dijkstar_->getPotentialMap(), map_x_size_,
  // map_y_size_);
  std::vector<Pose2d> path;
  float start_cost = ptr_dijkstar_->getCost(
      costCharMap, ptr_dijkstar_->toIndex(start_point.ui_x, start_point.ui_y));
  std::cout << "start cost = " << start_cost << std::endl;
  float target_cost = ptr_dijkstar_->getCost(
      costCharMap, ptr_dijkstar_->toIndex(end_point.ui_x, end_point.ui_y));
  std::cout << "target cost = " << target_cost << std::endl;

  bool path_result = ptr_dijkstar_->getPath(
      start_point.ui_x, start_point.ui_y, end_point.ui_x, end_point.ui_y, path);
  std::cout << "Path result = " << path_result << " with size = " << path.size()
            << std::endl;
  EXPECT_EQ(true, path_result);
  // for (unsigned int i = 0; i < path.size(); ++i) {
  //   std::cout << "(" << path[i].getX() << ", " << path[i].getY() << ") ->";
  // }
  std::cout << std::endl << std::endl;
  // }
}

}  // namespace CVTE_BABOT

CVTE_BABOT::DstarPlannerTest *dijkstar_tester = NULL;

TEST(DstarPlannerTest, test_costmap) {
  dijkstar_tester->testCostMap();
}

// TEST(DstarPlannerTest, test_potential) {
//   dijkstar_tester->tesetPotentialCalculator();
// }

// TEST(DstarPlannerTest, test_outlinemap) {
//   dijkstar_tester->testOutLineMap();
// }

// TEST(DstarPlannerTest, test_getcost) {
//   dijkstar_tester->testGetCost();
// }

// TEST(DstarPlannerTest, test_potentialcalc) {
//   dijkstar_tester->testPotentialCalc();
// }

// TEST(DstarPlannerTest, test_Dstarpotential) {
//   dijkstar_tester->testPotentials();
// }

TEST(DstarPlannerTest, test_getpath) {
  dijkstar_tester->testGetPath();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  dijkstar_tester = new CVTE_BABOT::DstarPlannerTest();

  return RUN_ALL_TESTS();
}

#endif  // TEST_D_STAR_PLANNER_TEST