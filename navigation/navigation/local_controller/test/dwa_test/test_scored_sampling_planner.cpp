/*
* brief : simple_scored_sampling_planner.cpp 文件的单元测试
* create: 2019-03-29
* tester: liangjiajun
*/

#ifndef TEST_SCORED_SAMPLING_PLANNER_HPP
#define TEST_SCORED_SAMPLING_PLANNER_HPP

#include <gtest/gtest.h>
#include <memory>
#include "local_planner_limits.hpp"
#include "map_grid_cost_function.hpp"
#include "obstacle_cost_function.hpp"
#include "oscillation_cost_function.hpp"
#include "simple_trajectory_generator.hpp"
#include "simple_trajectory_scorer.hpp"
#include "trajectory.hpp"

#include "costmap2d_builder.hpp"
#include "costmap2d_director.hpp"
#include "costmap_mediator.hpp"
#include "costmap_model.hpp"
#include "costmap_testing_helper.hpp"
#include "world_map_data.hpp"

namespace CVTE_BABOT {

class SimpleScoredSamplingTest : public testing::Test {
 public:
  virtual void TestBody() {}

  static void updateFunc(
      const std::shared_ptr<CVTE_BABOT::WorldmapData> &wm_data,
      std::shared_ptr<CVTE_BABOT::WorldmapData> &ptr_wm_data) {
    ptr_wm_data = std::make_shared<CVTE_BABOT::WorldmapData>(*wm_data);
  }

  SimpleScoredSamplingTest() {
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
    ptr_costmap_ =
        std::make_shared<Costmap2d>(*(ptr_costmap_mediator_->getCostmap()));

    auto builder = std::make_shared<Costmap2dBuilder>(ptr_layered_costmap_);
    auto director = std::make_shared<Costmap2dDirector>(builder);
    director->buildCostmap(layer_param_);

    ptr_layered_costmap_->updateMap(robot_current_pose_);

    ptr_costmap_ =
        std::make_shared<Costmap2d>(*(ptr_costmap_mediator_->getCostmap()));
  }

  void construct_test() {
    CostmapModel::getPtrInstance()->setPtrCostmap(ptr_costmap_);
    auto ob_cost_function_ptr = std::make_shared<ObstacleCostFunction>();
    ob_cost_function_ptr->setScale(1.0 * 0.2);

    auto map_grid_cost_function_ptr =
        std::make_shared<MapGridCostFunction>(0.0, 0.0, false);
    map_grid_cost_function_ptr->setScale(1.0 * 64.0 * 0.5);

    auto oscillation_cost_function_ptr =
        std::make_shared<OscillationCostFunction>();
    oscillation_cost_function_ptr->setOscillationResetDist(0.25, 0.1);
    oscillation_cost_function_ptr->resetOscillationFlags();

    std::vector<std::shared_ptr<CostFunction>> critics;
    critics.push_back(oscillation_cost_function_ptr);
    critics.push_back(map_grid_cost_function_ptr);
    critics.push_back(ob_cost_function_ptr);

    auto traj_generator_ptr = std::make_shared<SimpleTrajectoryGenerator>();

    auto score_sample_ptr =
        std::make_shared<SimpleTrajectoryScorer>(traj_generator_ptr, critics);

    EXPECT_NE(nullptr, score_sample_ptr);
  }

  void score_test() {
    CostmapModel::getPtrInstance()->setPtrCostmap(ptr_costmap_);
    auto ob_cost_function_ptr = std::make_shared<ObstacleCostFunction>();
    ob_cost_function_ptr->setScale(1.0 * 0.2);

    auto map_grid_cost_function_ptr =
        std::make_shared<MapGridCostFunction>(0.0, 0.0, false);
    map_grid_cost_function_ptr->setScale(1.0 * 64.0 * 0.5);

    auto oscillation_cost_function_ptr =
        std::make_shared<OscillationCostFunction>();
    oscillation_cost_function_ptr->setOscillationResetDist(0.25, 0.1);
    oscillation_cost_function_ptr->resetOscillationFlags();

    std::vector<std::shared_ptr<CostFunction>> critics;
    critics.push_back(oscillation_cost_function_ptr);
    critics.push_back(map_grid_cost_function_ptr);
    critics.push_back(ob_cost_function_ptr);

    auto traj_generator_ptr = std::make_shared<SimpleTrajectoryGenerator>();

    typedef struct {
      double x;
      double y;
      double theta;
    } test_params;

    test_params test[] = {{2.0, 3.0, 0.4},    {-2.0, 3.0, -0.4},
                          {2.0, -3.0, 4.4},   {2.0, -3.0, 4.4},
                          {20.0, -30.0, 4.4}, {-10.0, -0.0, 0.0},
                          {0.0, 3.0, 2.4}};

    Eigen::Vector3f vel(0.0, 0.0, 0.0);
    Eigen::Vector3f goal(1.0, 1.0, 0.0);
    Eigen::Vector3f zero_vsample(1.0, 1.0, 1.0);
    bool discretize_by_time = true;
    auto lp_limit_ptr = std::make_shared<LocalPlannerLimits>(
        1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 1.0, 0.0, 1.0, 3.2, 0.4, 0.4,
        true, 0.01, 0.01);
    auto simple_gen_ptr = std::make_shared<SimpleTrajectoryGenerator>();

    simple_gen_ptr->setParameters(1.0, 0.05, 0.05, true, 0.05);
    Trajectory traj_result;
    Trajectory best_result;
    auto traj_vector_ptr = std::make_shared<std::vector<Trajectory>>();
    for (auto test_pose : test) {
      Eigen::Vector3f pose(test_pose.x, test_pose.y, test_pose.theta);
      simple_gen_ptr->initialise(pose, vel, goal, lp_limit_ptr, zero_vsample,
                                 discretize_by_time);
      for (unsigned int i = 0; i < simple_gen_ptr->sample_params_.size(); i++) {
        auto value = simple_gen_ptr->sample_params_[i];
        simple_gen_ptr->generateTrajectory(pose, vel, value, traj_result);
        traj_vector_ptr->push_back(traj_result);
      }
    }

    auto score_sample_ptr =
        std::make_shared<SimpleTrajectoryScorer>(traj_generator_ptr, critics);

    for (unsigned int i = 0; i < traj_vector_ptr->size(); ++i) {
      double score =
          score_sample_ptr->scoreTrajectory(traj_vector_ptr->at(i), -1);
      // std::cout << "score = " << score;
      EXPECT_GE(0.0, score);
    }
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

TEST(SampleScoreTest, construct_test) {
  SimpleScoredSamplingTest sample_tester;
  sample_tester.construct_test();
}

TEST(SampleScoreTest, score_test) {
  SimpleScoredSamplingTest sample_tester;
  sample_tester.score_test();
}

}  // namespace CVTE_BABOT

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

#endif
