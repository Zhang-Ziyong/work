/*
* brief : traject.cpp 文件的单元测试
* create: 2019-04-11
* tester: caoyong
*/

#ifndef TEST_TRAJECT_HPP
#define TEST_TRAJECT_HPP

#include <gtest/gtest.h>
#include <memory>
#include "trajectory.hpp"

namespace CVTE_BABOT {

class TrajectTest : public testing::Test {
 public:
  virtual void TestBody() {}

  /**
*@brief 测试说明
*１．测试路径的构造和复制
* */
  void construct_traject_test() {
    auto traject_ptr = std::make_shared<Trajectory>(1.0, 1.0, 0.5, 0.1, 10);
    auto traject_def_ptr = std::make_shared<Trajectory>();
    auto traject_cp_ptr = traject_ptr;
    EXPECT_NE(nullptr, traject_ptr.get());
    EXPECT_NE(nullptr, traject_def_ptr.get());
    EXPECT_NE(nullptr, traject_cp_ptr.get());

    EXPECT_EQ(traject_cp_ptr->xv_, traject_ptr->xv_);
    EXPECT_EQ(traject_cp_ptr->yv_, traject_ptr->yv_);
    EXPECT_EQ(traject_cp_ptr->thetav_, traject_ptr->thetav_);
    EXPECT_EQ(traject_cp_ptr->time_delta_, traject_ptr->time_delta_);
  }

  /**
*@brief 测试说明
*１．测试路径点的插入和查询
*2. 针对三个参数的多态函数
* */
  void point_checker_test_a() {
    typedef struct {
      int index;
      double x;
      double y;
      double th;
    } traj_params;

    traj_params test_table[] = {{
                                    1, 0.0, 1.0, 0.5,
                                },
                                {2, -111.0, 0.5, -0.5},
                                {3, -20.0, -0.5, -0.5},
                                {4, 100.0, -99.5, -0.5},
                                {5, -2.0, 0.5, 1.5},
                                {6, 2.4, -0.5, 0.5},
                                {7, -0.0, 0.35, -0.0},
                                {8, 99.0, 0.0, 1.0}};

    for (auto value : test_table) {
      auto traject_ptr = std::make_shared<Trajectory>(1.0, 1.0, 0.5, 0.1, 10);
      traject_ptr->setPoint(value.index, value.x, value.y, value.th);
      traj_params result;
      traject_ptr->getPoint(result.index, result.x, result.y, result.th);
      EXPECT_EQ(value.index, result.index);
      EXPECT_EQ(value.x, result.x);
      EXPECT_EQ(value.y, result.y);
      EXPECT_EQ(value.th, result.th);

      auto traject_def_ptr = std::make_shared<Trajectory>();
      traject_def_ptr->addPoint(value.x, value.y, value.th);
      traject_def_ptr->getEndpoint(result.x, result.y, result.th);
      EXPECT_EQ(value.index, result.index);
      EXPECT_EQ(value.x, result.x);
      EXPECT_EQ(value.y, result.y);
      EXPECT_EQ(value.th, result.th);
    }
  }

  /**
*@brief 测试说明
*１．测试路径点的插入和查询
*2. 针对七个参数的多态函数
* */
  void point_checker_test_b() {
    typedef struct {
      int index;
      double x;
      double y;
      double th;
      double vx;
      double vy;
      double vth;
    } traj_params;

    traj_params test_table[] = {{1, 1.0, 1.0, 0.5, -0.5, 0.0, 0.0},
                                {2, -1.0, 0.5, -0.5, 0.5, 0.0, 0.0},
                                {3, 0.0, -0.5, -0.5, 0.5, 0.0, 2.0},
                                {4, 100.0, -99.5, -0.5, 0.5, -19.0, -3.0},
                                {5, 2.0, 0.5, 1.5, 0.5, 2.0, 3.0}};

    for (auto value : test_table) {
      auto traject_def_ptr =
          std::make_shared<Trajectory>(1.0, 1.0, 0.5, 0.1, 10);
      traject_def_ptr->setPoint(value.index, value.x, value.y, value.th,
                                value.vx, value.vy, value.vth);
      traj_params result;
      traject_def_ptr->getPoint(result.index, result.x, result.y, result.th,
                                result.vx, result.vy, result.vth);
      EXPECT_EQ(value.index, result.index);
      EXPECT_EQ(value.x, result.x);
      EXPECT_EQ(value.y, result.y);
      EXPECT_EQ(value.th, result.th);
      EXPECT_EQ(value.vx, result.vx);
      EXPECT_EQ(value.vy, result.vy);
      EXPECT_EQ(value.vth, result.vth);

      auto traject_ptr = std::make_shared<Trajectory>();
      traject_ptr->addPoint(value.x, value.y, value.th, value.vx, value.vy,
                            value.vth);
      traject_ptr->getEndpoint(result.x, result.y, result.th, result.vx,
                               result.vy, result.vth);
      EXPECT_EQ(value.index, result.index);
      EXPECT_EQ(value.x, result.x);
      EXPECT_EQ(value.y, result.y);
      EXPECT_EQ(value.th, result.th);
      EXPECT_EQ(value.vx, result.vx);
      EXPECT_EQ(value.vy, result.vy);
      EXPECT_EQ(value.vth, result.vth);
    }
  }

  /**
*@brief 测试说明
*１．测试路径点的插入和查询
*2. 针对十个参数的多态函数
* */
  void point_checker_test_c() {
    typedef struct {
      int index;
      double x;
      double y;
      double th;
      double vx;
      double vy;
      double vth;
      double accX;
      double accY;
      double accTh;
    } traj_params;

    traj_params test_table[] = {
        {1, 1.0, 1.0, 0.5, -0.5, 0.0, 0.0, 1.0, 2.0, 3.0},
        {2, -1.0, 0.5, -0.5, 0.5, 0.0, 0.0, -1.0, -2.0, -3.0},
        {3, 0.0, -0.5, -0.5, 0.5, 0.0, 2.0, 0.0, -1.0, -100},
        {4, 100.0, -99.5, -0.5, 0.5, -19.0, -3.0, -11, -2.0, 1.0},
        {5, 2.0, 0.5, 1.5, 0.5, 2.0, 3.0, 2.0, -0.001, -100}};

    for (auto value : test_table) {
      auto traject_def_ptr =
          std::make_shared<Trajectory>(1.0, 1.0, 0.5, 0.1, 10);
      traject_def_ptr->setPoint(value.index, value.x, value.y, value.th,
                                value.vx, value.vy, value.vth, value.accX,
                                value.accY, value.accTh);
      traj_params result;
      traject_def_ptr->getPoint(result.index, result.x, result.y, result.th,
                                result.vx, result.vy, result.vth, result.accX,
                                result.accY, result.accTh);
      EXPECT_EQ(value.index, result.index);
      EXPECT_EQ(value.x, result.x);
      EXPECT_EQ(value.y, result.y);
      EXPECT_EQ(value.th, result.th);
      EXPECT_EQ(value.vx, result.vx);
      EXPECT_EQ(value.vy, result.vy);
      EXPECT_EQ(value.vth, result.vth);
      EXPECT_EQ(value.accX, result.accX);
      EXPECT_EQ(value.accY, result.accY);
      EXPECT_EQ(value.accTh, result.accTh);

      auto traject_ptr = std::make_shared<Trajectory>();
      traject_ptr->addPoint(value.x, value.y, value.th, value.vx, value.vy,
                            value.vth, value.accX, value.accY, value.accTh);
      traject_ptr->getEndpoint(result.x, result.y, result.th, result.vx,
                               result.vy, result.vth, result.accX, result.accY,
                               result.accTh);
      EXPECT_EQ(value.index, result.index);
      EXPECT_EQ(value.x, result.x);
      EXPECT_EQ(value.y, result.y);
      EXPECT_EQ(value.th, result.th);
      EXPECT_EQ(value.vx, result.vx);
      EXPECT_EQ(value.vy, result.vy);
      EXPECT_EQ(value.vth, result.vth);
      EXPECT_EQ(value.accX, result.accX);
      EXPECT_EQ(value.accY, result.accY);
      EXPECT_EQ(value.accTh, result.accTh);
    }
  }
};

TEST(TrajectTest, construct_test) {
  TrajectTest traj_test;
  traj_test.construct_traject_test();
}

TEST(TrajectTest, point_test_a) {
  TrajectTest traj_test;
  traj_test.point_checker_test_a();
}

TEST(TrajectTest, point_test_b) {
  TrajectTest traj_test;
  traj_test.point_checker_test_b();
}

TEST(TrajectTest, point_test_c) {
  TrajectTest traj_test;
  traj_test.point_checker_test_c();
}

}  // namespace CVTE_BABOT

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

#endif