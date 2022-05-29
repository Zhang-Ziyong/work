/**
 * @file hdmap_impl_test.cpp
 * @author linyanlong(linyanlong@cvte.com)
 * @brief 
 * @version 0.1
 * @date 2022-01-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "hdmap_impl.hpp"
#include "gtest/gtest.h"
namespace cvte {
namespace hdmap {
class HdMapImplTestSuite : public ::testing::Test {
 public:
  HdMapImplTestSuite() {
    
  }

 public:
  HdMapImpl hdmap_impl_;
};
}
}