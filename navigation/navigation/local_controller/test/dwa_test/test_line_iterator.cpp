/*
 * brief : line_iterator.hpp 文件的单元测试
 *
 * create: 2019-04-13
 *
 * tester: caoyong
 */

#ifndef TEST_LINE_ITERATOR_HPP
#define TEST_LINE_ITERATOR_HPP

#include <gtest/gtest.h>
#include "line_iterator.hpp"

namespace CVTE_BABOT {

/**
 * @brief 测试说明
 * １．测试直线迭代器的构造
 * ２．测试直线迭代器的南端延长
 * */

TEST(LineIterator, south) {
  LineIterator line(1, 2, 1, 4);
  EXPECT_TRUE(line.isValid());
  EXPECT_EQ(1, line.getX());
  EXPECT_EQ(2, line.getY());
  line.advance();
  EXPECT_TRUE(line.isValid());
  EXPECT_EQ(1, line.getX());
  EXPECT_EQ(3, line.getY());
  line.advance();
  EXPECT_TRUE(line.isValid());
  EXPECT_EQ(1, line.getX());
  EXPECT_EQ(4, line.getY());
  line.advance();
  EXPECT_FALSE(line.isValid());
}

/**
 * @brief 测试说明
 * １．测试直线迭代器的构造
 * ２．测试直线迭代器的其它方向延长
 * */

TEST(LineIterator, north_north_west) {
  LineIterator line(0, 0, -2, -4);
  EXPECT_TRUE(line.isValid());
  EXPECT_EQ(0, line.getX());
  EXPECT_EQ(0, line.getY());
  line.advance();
  EXPECT_TRUE(line.isValid());
  EXPECT_EQ(-1, line.getX());
  EXPECT_EQ(-1, line.getY());
  line.advance();
  EXPECT_TRUE(line.isValid());
  EXPECT_EQ(-1, line.getX());
  EXPECT_EQ(-2, line.getY());
  line.advance();
  EXPECT_TRUE(line.isValid());
  EXPECT_EQ(-2, line.getX());
  EXPECT_EQ(-3, line.getY());
  line.advance();
  EXPECT_TRUE(line.isValid());
  EXPECT_EQ(-2, line.getX());
  EXPECT_EQ(-4, line.getY());
  line.advance();
  EXPECT_FALSE(line.isValid());
}

}  // namespace CVTE_BABOT

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

#endif
