/*
 * brief : velocity_iterator.hpp 文件的单元测试
 *
 * create: 2019-04-11
* tester: caoyong
 */

#ifndef TEST_VELOCITY_ITERATOR_HPP
#define TEST_VELOCITY_ITERATOR_HPP

#include <gtest/gtest.h>
#include "velocity_iterator.hpp"

namespace CVTE_BABOT {

/**
 * @brief 测试说明
 * １．速度迭代器的速度采样
 * */

TEST(VelocityIteratorTest, testsingle) {
  double result[5];
  int i = 0;
  for (VelocityIterator x_it(0.0, 0.0, 10); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(1, i);
  EXPECT_EQ(0, result[0]);
}

/**
 * @brief 测试说明
 * １．速度迭代器的速度采样，正值
 * */

TEST(VelocityIteratorTest, testsingle_pos) {
  double result[5];
  int i = 0;
  for (VelocityIterator x_it(2.2, 2.2, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(1, i);
  EXPECT_EQ(2.2, result[0]);
}

/**
 * @brief 测试说明
 * １．速度迭代器的速度采样，负值
 * */

TEST(VelocityIteratorTest, testsingle_neg) {
  double result[5];
  int i = 0;
  for (VelocityIterator x_it(-3.3, -3.3, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(1, i);
  EXPECT_EQ(-3.3, result[0]);
}

/**
 * @brief 测试说明
 * １．速度迭代器的速度采样，负到正值
 * */

TEST(VelocityIteratorTest, test1) {
  double result[5];
  int i = 0;
  for (VelocityIterator x_it(-30, 30, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(3, i);
  double expected[3] = {-30.0, 0.0, 30.0};
  for (int j = 0; j < 3; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

/**
 * @brief 测试说明
 * １．速度迭代器的速度采样，正值步长为１
 * */

TEST(VelocityIteratorTest, test1_pos) {
  double result[5];
  int i = 0;
  for (VelocityIterator x_it(10, 30, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(2, i);
  double expected[2] = {10.0, 30.0};
  for (int j = 0; j < 2; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

/**
 * @brief 测试说明
 * １．速度迭代器的速度采样，负值步长为１
 * */

TEST(VelocityIteratorTest, test1_neg) {
  double result[5];
  int i = 0;
  for (VelocityIterator x_it(-30, -10, 1); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(2, i);
  double expected[2] = {-30.0, -10.0};
  for (int j = 0; j < 2; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

/**
 * @brief 测试说明
 * １．速度迭代器的速度采样，负值步长为３
 * */

TEST(VelocityIteratorTest, test3) {
  double result[5];
  int i = 0;
  for (VelocityIterator x_it(-30, 30, 3); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(3, i);
  double expected[3] = {-30.0, 0.0, 30};
  for (int j = 0; j < 3; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

/**
 * @brief 测试说明
 * １．速度迭代器的速度采样，-30到 30的步长为４
 * */

TEST(VelocityIteratorTest, test4) {
  double result[5];
  int i = 0;
  for (VelocityIterator x_it(-30, 30, 4); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(5, i);
  double expected[5] = {-30.0, -10.0, 0.0, 10.0, 30};
  for (int j = 0; j < 5; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

/**
 * @brief 测试说明
 * １．速度迭代器的速度采样，-１0到 ５０的步长为４
 * */

TEST(VelocityIteratorTest, test_shifted) {
  double result[5];
  int i = 0;
  for (VelocityIterator x_it(-10, 50, 4); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(5, i);
  double expected[5] = {-10.0, 0.0, 10.0, 30, 50};
  for (int j = 0; j < 5; ++j) {
    EXPECT_EQ(expected[j], result[j]);
  }
}

/**
 * @brief 测试说明
 * １．速度迭代器的速度采样，浮点数类型
 * */

TEST(VelocityIteratorTest, test_cranky) {
  double result[4];
  int i = 0;
  for (VelocityIterator x_it(-10.00001, 10, 3); !x_it.isFinished(); x_it++) {
    result[i] = x_it.getVelocity();
    i++;
  }
  EXPECT_EQ(4, i);
  for (int j = 0; j < 4; ++j) {
    double expected[5] = {-10.00001, -0.000005, 0.0, 10.0};
    EXPECT_FLOAT_EQ(expected[j], result[j]);
  }
}

}  // namespace CVTE_BABOT

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

#endif
