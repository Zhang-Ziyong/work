/*
* brief : simple_trajectory_generator.cpp 文件的单元测试
* create: 2019-04-12
* tester: caoyong
*/

#ifndef TEST_SIMPLE_TRAJECTORY_GENERATOR_HPP
#define TEST_SIMPLE_TRAJECTORY_GENERATOR_HPP

#include <gtest/gtest.h>
#include <memory>
#include "local_planner_limits.hpp"
#include "simple_trajectory_generator.hpp"
#include "trajectory.hpp"

namespace CVTE_BABOT {

class SimpleTrajectoryGeneratorTest : public testing::Test {
 public:
  virtual void TestBody() {}

  /**
*@brief 测试说明
*１．初始化测试
* */
  void initialise_test() {
    Eigen::Vector3f pose(1.0, 1.0, 0.0);
    Eigen::Vector3f vel(2.0, 2.0, 0.0);
    Eigen::Vector3f goal(10.0, 10.0, 0.0);
    Eigen::Vector3f zero_vsample(2.0, 2.0, 0.0);
    bool discretize_by_time = true;
    auto lp_limit_ptr = std::make_shared<LocalPlannerLimits>(
        1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 1.0, 0.0, 1.0, 3.2, 0.4, 0.4,
        true, 0.01, 0.01);
    auto simple_gen_ptr = std::make_shared<SimpleTrajectoryGenerator>();
    simple_gen_ptr->initialise(pose, vel, goal, lp_limit_ptr, zero_vsample,
                               discretize_by_time);
    simple_gen_ptr->setParameters(1.2, 0.05, 0.05, true, 0.05);
    EXPECT_EQ(1.2, simple_gen_ptr->sim_time_);
    EXPECT_EQ(0.05, simple_gen_ptr->sim_granularity_);
    EXPECT_EQ(0.05, simple_gen_ptr->angular_sim_granularity_);
    EXPECT_EQ(true, simple_gen_ptr->use_dwa_);
    EXPECT_EQ(0.05, simple_gen_ptr->sim_period_);
    EXPECT_EQ(0, simple_gen_ptr->sample_params_.size());

    Eigen::Vector3f vsample(2.0, 2.0, 1.0);
    simple_gen_ptr->initialise(pose, vel, goal, lp_limit_ptr, vsample,
                               discretize_by_time);
    EXPECT_NE(0, simple_gen_ptr->sample_params_.size());

    for (unsigned int i = 0; i < simple_gen_ptr->sample_params_.size(); i++) {
      auto value = simple_gen_ptr->sample_params_[i];
      bool vel_sample_a = (0 <= value[0] && value[0] <= vel[0]);
      bool vel_sample_b = (0 <= value[1] && value[1] <= vel[0]);
      EXPECT_EQ(true, vel_sample_a);
      EXPECT_EQ(true, vel_sample_b);
    }
  }

  /**
*@brief 测试说明
*１．生成轨迹测试
* */
  void generator_test() {
    typedef struct {
      double x;
      double y;
      double theta;
    } test_params;

    test_params test[] = {{2.0, 3.0, 0.4},    {-2.0, 3.0, -0.4},
                          {2.0, -3.0, 4.4},   {2.0, -3.0, 4.4},
                          {20.0, -30.0, 4.4}, {-10.0, -0.0, 0.0},
                          {0.0, 3.0, 2.4}};

    Eigen::Vector3f vel(3.0, 2.0, 0.0);
    Eigen::Vector3f goal(10.0, 10.0, 0.0);
    Eigen::Vector3f zero_vsample(2.0, 2.0, 1.0);
    bool discretize_by_time = true;
    auto lp_limit_ptr = std::make_shared<LocalPlannerLimits>(
        1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 1.0, 0.0, 1.0, 3.2, 0.4, 0.4,
        true, 0.01, 0.01);
    auto simple_gen_ptr = std::make_shared<SimpleTrajectoryGenerator>();

    simple_gen_ptr->setParameters(2.0, 0.05, 0.05, true, 0.05);
    Trajectory traj_result;
    for (auto test_pose : test) {
      Eigen::Vector3f pose(test_pose.x, test_pose.y, test_pose.theta);
      simple_gen_ptr->initialise(pose, vel, goal, lp_limit_ptr, zero_vsample,
                                 discretize_by_time);
      for (unsigned int i = 0; i < simple_gen_ptr->sample_params_.size(); i++) {
        auto value = simple_gen_ptr->sample_params_[i];
        simple_gen_ptr->generateTrajectory(pose, vel, value, traj_result);
      }
      //   std::cout << "resulet_size = " << traj_result.getPointsSize()
      //             << std::endl;

      // 采样后的路径会比原来的更密集
      EXPECT_LE(simple_gen_ptr->sample_params_.size(),
                traj_result.getPointsSize());

      for (unsigned int i = 0; i < traj_result.getPointsSize(); ++i) {
        // std::cout << traj_result.x_pts_[i] << " " << traj_result.y_pts_[i]
        //           << " "
        //           << " " << traj_result.th_pts_[i] << " "
        //           << traj_result.Vx_pts_[i] << " " << traj_result.Vy_pts_[i]
        //           << " " << traj_result.Vth_pts_[i] << std::endl;
        EXPECT_NE(0.0, traj_result.x_pts_[i]);
        EXPECT_NE(0.0, traj_result.y_pts_[i]);
        EXPECT_NE(0.0, traj_result.th_pts_[i]);
      }
    }
  }

  /**
*@brief 测试说明
*１．计算位姿测试
* */
  void compute_pose_test() {
    typedef struct {
      double x;
      double y;
      double theta;
      double v_x;
      double v_y;
      double v_theta;
      double dt;
    } test_params;

    test_params test[] = {{2.0, 3.0, 0.4, 12.0, -3.0, 0.4, 0.05},
                          {12.0, 13.0, -0.4, 0.0, -3.0, 0.4, 0.05},
                          {-2.0, 0.0, 0.0, 1.0, -3.0, 0.4, 0.05},
                          {0.0, -3.0, 1.0, 10.0, -3.0, 0.4, 0.05},
                          {10.0, -10.0, 2.0, 20.0, -3.0, 0.4, 0.05},
                          {20.0, -20.0, -0.5, 12.0, -3.0, 0.4, 0.05},
                          {30.0, -30.0, 0.4, 12.0, -3.0, 0.4, 0.05}};

    auto traj_ptr = std::make_shared<SimpleTrajectoryGenerator>();
    for (auto params : test) {
      Eigen::Vector3f pose(params.x, params.y, params.theta);
      Eigen::Vector3f vel(params.v_x, params.v_y, params.v_theta);
      Eigen::Vector3f new_pose =
          traj_ptr->computeNewPositions(pose, vel, params.dt);
      //   std::cout << new_pose[0] << " " << new_pose[1] << " " << new_pose[2]
      //             << std::endl;
      EXPECT_NE(0.0, new_pose[0]);
      EXPECT_NE(0.0, new_pose[1]);
      EXPECT_NE(0.0, new_pose[2]);
    }
  }

  /**
*@brief 测试说明
*１．计算速度测试
* */
  void compute_velocity_test() {
    typedef struct {
      double x;
      double y;
      double theta;
      double v_x;
      double v_y;
      double v_theta;
      double acc_x;
      double acc_y;
      double acc_theta;
      double dt;
    } test_params;

    test_params test[] = {
        {2.0, 3.0, 0.4, 12.0, -3.0, 0.4, 1.0, 0.0, 0.0, 0.05},
        {12.0, 13.0, -0.4, 0.0, -3.0, 0.4, 1.0, 1.0, 0.0, 0.05},
        {-2.0, 0.0, 0.0, 1.0, -3.0, 0.4, 1.0, 1.0, 1.0, 0.05},
        {0.0, -3.0, 1.0, 10.0, -3.0, 0.4, 0.5, 1.0, 0.0, 0.05},
        {10.0, -10.0, 2.0, 20.0, -3.0, 0.4, 1.0, 0.5, 0.0, 0.01},
        {20.0, -20.0, -0.5, 12.0, -3.0, 0.4, 1.0, 1.0, 0.5, 0.01},
        {30.0, -30.0, 0.4, 12.0, -3.0, 0.4, 0.0, 1.0, 0.0, 0.01}};

    auto traj_ptr = std::make_shared<SimpleTrajectoryGenerator>();
    for (auto params : test) {
      Eigen::Vector3f target_vel(params.x, params.y, params.theta);
      Eigen::Vector3f vel(params.v_x, params.v_y, params.v_theta);
      Eigen::Vector3f acclimit(params.acc_x, params.acc_y, params.acc_theta);
      Eigen::Vector3f new_velocity =
          traj_ptr->computeNewVelocities(target_vel, vel, acclimit, params.dt);
      //   std::cout << target_vel[0] << " " << vel[0] << " " << acclimit[0]
      //             << std::endl;
      //   std::cout << new_velocity[0] << " " << new_velocity[1] << " "
      //             << new_velocity[2] << std::endl;
      EXPECT_NE(0.0, new_velocity[0]);
      EXPECT_NE(0.0, new_velocity[1]);
      EXPECT_NE(0.0, new_velocity[2]);
    }
  }
};

TEST(SimpleTrajectGeneratorTest, init_test) {
  SimpleTrajectoryGeneratorTest traj_tester;
  traj_tester.initialise_test();
}

TEST(SimpleTrajectGeneratorTest, generator_test) {
  SimpleTrajectoryGeneratorTest traj_tester;
  traj_tester.generator_test();
}

TEST(SimpleTrajectGeneratorTest, compute_pose_test) {
  SimpleTrajectoryGeneratorTest traj_tester;
  traj_tester.compute_pose_test();
}

TEST(SimpleTrajectGeneratorTest, compute_velocity_test) {
  SimpleTrajectoryGeneratorTest traj_tester;
  traj_tester.compute_velocity_test();
}

}  // namespace CVTE_BABOT

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
#endif