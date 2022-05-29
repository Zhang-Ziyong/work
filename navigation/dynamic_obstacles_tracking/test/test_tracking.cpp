/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file test_tracking.cpp
 *
 *@brief cpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.7
 *@data 2020-07-10
 ************************************************************************/
#include <glog/logging.h>
#include <gtest/gtest.h>

// #include "common/bounding_box.hpp"
#include "common/bounding.hpp"
#include "common/tracking_utils.hpp"
namespace CVTE_BABOT {

class TrackingTester : public testing::Test {
 public:
  virtual void TestBody() {}
  TrackingTester() {}

  bool ifDynamicObj() { return true; }

  void dynamicObjTest() {
    ObjectPtr ptr_obj1 = std::make_shared<Object>();
    ptr_obj1->width = 1.92624;
    ptr_obj1->length = 3.72674;
    ptr_obj1->height = 1.0;
    ptr_obj1->object_center(0) = 13.6677;
    ptr_obj1->object_center(1) = -22.8762;
    ptr_obj1->object_center(2) = 0.00;
    GroundBox gbox1;
    toGroundBox(ptr_obj1, &gbox1);

    ObjectPtr ptr_obj2 = std::make_shared<Object>();
    ptr_obj2->width = 0.3;
    ptr_obj2->length = 0.331903;
    ptr_obj2->height = 1.0;
    // ptr_obj2->object_center(0) = 1.00;
    // ptr_obj2->object_center(1) = 1.00;
    // ptr_obj2->object_center(2) = 1.00;
    ptr_obj2->object_center(0) = 12.3683;
    ptr_obj2->object_center(1) = -17.1964;
    ptr_obj2->object_center(2) = 0.00;
    GroundBox gbox2;
    toGroundBox(ptr_obj2, &gbox2);

    double threshold = 0.5;
    bool over = groundBoxOverlap(gbox1, gbox2, threshold);
    bool in1 = aInsideB(gbox1, gbox2);
    bool in2 = aInsideB(gbox2, gbox1);
    // LOG(INFO) << "groundBoxIoU: " << groundBoxIoU(gbox1, gbox2);
    // LOG(INFO) << "aInsideB1: " << in1;
    // LOG(INFO) << "aInsideB2: " << in2;
    // LOG(INFO) << "groundBoxOverlap: " << over;
  }

 private:
};  // namespace CVTE_BABOT

TEST(TrackingTester, rotateRecoveryTest) {
  TrackingTester tracking_tester;
  tracking_tester.dynamicObjTest();
}

}  // namespace CVTE_BABOT

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
