/************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2019, CVTE.
* All rights reserved.
*
*@file test_message_filter.cpp
*
*@brief 测试message_filter.cpp 文件

*@author wuhuabo <wuhuabo@cvte.com>
*
*@modified wuhuabo <wuhuabo@cvte.com>
*@version current_algo.dev.1.0
*@data 2019-05-25
************************************************************************/

#include <gtest/gtest.h>
#include "message_filter/message_filter.hpp"

namespace CVTE_BABOT {

class MessageFilterTester : public testing::Test {
 public:
  virtual void TestBody() {}

  void laserScanCallback(MessageFilter<double>::PtrMessage, PtrTransform) {
    call_time_++;
  }

  void handleTransformableMsgTest() {
    call_time_ = 0;
    double tolerance = 0.05;
    auto ptr_scan_mf =
        std::make_shared<MessageFilter<double>>(tolerance, 0.5, 50, 50);
    ptr_scan_mf->registerTransformableCB(
        std::bind(&MessageFilterTester::laserScanCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    PtrTransform ptr_transform = std::make_shared<Transform>();
    for (auto it : {1.06, 1.05, 1.04, 1.03, 1.02, 1.01}) {
      ptr_scan_mf->l_messageInfo.push_back(
          MessageFilter<double>::MessageInfo(it, std::make_shared<double>(it)));
    }

    // time diff greater than tolerance
    ASSERT_FALSE(
        ptr_scan_mf->handleTransformableMsg(1.06 + tolerance, ptr_transform));

    ASSERT_FALSE(
        ptr_scan_mf->handleTransformableMsg(1.01 - tolerance, ptr_transform));

    // diff less than the second message
    ASSERT_TRUE(ptr_scan_mf->handleTransformableMsg(1.06 - tolerance - 0.01,
                                                    ptr_transform));
    // call when the first mesage matching and not continuing
    ASSERT_EQ(call_time_, 1);

    // erase the later message after the second message
    ASSERT_EQ(ptr_scan_mf->l_messageInfo.size(), 2);
  }

  void inputTransformTest() {
    call_time_ = 0;
    double tolerance = 0.05;
    auto ptr_scan_mf =
        std::make_shared<MessageFilter<double>>(tolerance, 0.5, 50, 50);
    ptr_scan_mf->registerTransformableCB(
        std::bind(&MessageFilterTester::laserScanCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    PtrTransform ptr_transform = std::make_shared<Transform>();
    for (auto it : {1.06, 1.05, 1.04, 1.03, 1.02, 1.01}) {
      ptr_scan_mf->l_messageInfo.push_back(
          MessageFilter<double>::MessageInfo(it, std::make_shared<double>(it)));
    }

    for (unsigned int i = 0; i < 48; i++) {
      ptr_scan_mf->l_transformInfo.push_back(
          MessageFilter<double>::TransformInfo(i,
                                               std::make_shared<Transform>()));
    }

    // time diff greater than tolerance
    ptr_scan_mf->inputTransform(1.06 + tolerance, ptr_transform);
    ASSERT_EQ(ptr_scan_mf->l_transformInfo.size(), 49);
    ASSERT_DOUBLE_EQ(ptr_scan_mf->l_transformInfo.front().time,
                     1.06 + tolerance);

    ptr_scan_mf->inputTransform(1.06 + tolerance + 0.1, ptr_transform);
    ASSERT_EQ(ptr_scan_mf->l_transformInfo.size(), 50);
    ASSERT_DOUBLE_EQ(ptr_scan_mf->l_transformInfo.front().time,
                     1.06 + tolerance + 0.1);

    // size remain 50
    ptr_scan_mf->inputTransform(1.06 + tolerance + 0.2, ptr_transform);
    ASSERT_EQ(ptr_scan_mf->l_transformInfo.size(), 50);
    ASSERT_DOUBLE_EQ(ptr_scan_mf->l_transformInfo.front().time,
                     1.06 + tolerance + 0.2);
    ASSERT_DOUBLE_EQ(ptr_scan_mf->l_transformInfo.back().time, 46.0);

    // diff less than the second message
    ptr_scan_mf->inputTransform(1.06 - tolerance - 0.01, ptr_transform);
    // success matching
    ASSERT_EQ(call_time_, 1);
    // erase the other transform
    ASSERT_EQ(ptr_scan_mf->l_transformInfo.size(), 1);
    ASSERT_DOUBLE_EQ(ptr_scan_mf->l_transformInfo.front().time,
                     1.06 - tolerance - 0.01);
  }

  void getLatestTransformTest() {
    call_time_ = 0;
    double tolerance = 2;
    double cache = 2;
    auto ptr_scan_mf =
        std::make_shared<MessageFilter<double>>(tolerance, cache, 50, 50);
    ptr_scan_mf->registerTransformableCB(
        std::bind(&MessageFilterTester::laserScanCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    for (int i = 50; i > 0; i--) {
      auto ptr_trans = std::make_shared<Transform>();
      ptr_trans->d_x = i;
      ptr_scan_mf->l_transformInfo.push_back(
          MessageFilter<double>::TransformInfo(i, ptr_trans));
    }

    PtrTransform ptr_transform = nullptr;
    // time diff greater than tolerance, wait for feasible
    ASSERT_EQ(ptr_scan_mf->getLatestTransform(50 + tolerance, ptr_transform),
              ptr_scan_mf->WaitForFeasible);

    // l_transformInfo not be changed, ptr_transform not be set
    ASSERT_EQ(ptr_scan_mf->l_transformInfo.size(), 50);
    ASSERT_EQ(ptr_transform, nullptr);

    // the best matching transform at the middle
    ASSERT_EQ(ptr_scan_mf->getLatestTransform(41 - 0.1, ptr_transform),
              ptr_scan_mf->CanTransform);

    ASSERT_EQ(ptr_scan_mf->l_transformInfo.size(), 10);
    ASSERT_DOUBLE_EQ(ptr_transform->d_x, 41);

    // the best matching transform at the end, not be erase last time
    ASSERT_EQ(ptr_scan_mf->getLatestTransform(41 - 0.1, ptr_transform),
              ptr_scan_mf->CanTransform);

    ASSERT_EQ(ptr_scan_mf->l_transformInfo.size(), 10);
    ASSERT_DOUBLE_EQ(ptr_transform->d_x, 41);

    // the latest transform later than message too much
    ptr_transform = nullptr;
    ASSERT_EQ(
        ptr_scan_mf->getLatestTransform(50 + cache + tolerance, ptr_transform),
        ptr_scan_mf->OutOfCache);
    ASSERT_EQ(ptr_scan_mf->l_transformInfo.size(), 0);
    ASSERT_EQ(ptr_transform, nullptr);

    // not transform now so wait for feasible
    ASSERT_EQ(ptr_scan_mf->getLatestTransform(50 + cache + 1, ptr_transform),
              ptr_scan_mf->WaitForFeasible);
  }
  void handleMessageTest() {
    call_time_ = 0;
    double tolerance = 2;
    double cache = 3;
    auto ptr_scan_mf =
        std::make_shared<MessageFilter<double>>(tolerance, cache, 50, 50);
    ptr_scan_mf->registerTransformableCB(
        std::bind(&MessageFilterTester::laserScanCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    for (int i = 49; i > 0; i--) {
      ptr_scan_mf->l_messageInfo.push_back(
          MessageFilter<double>::MessageInfo(i, std::make_shared<double>(i)));
    }

    for (int i = 50; i > 0; i--) {
      ptr_scan_mf->l_transformInfo.push_back(
          MessageFilter<double>::TransformInfo(i,
                                               std::make_shared<Transform>()));
    }

    // out of tolerance
    auto ptr_message = std::make_shared<double>(52);
    ptr_scan_mf->handleMessage(52.0, ptr_message);
    ASSERT_EQ(ptr_scan_mf->l_messageInfo.size(), 50);
    ASSERT_DOUBLE_EQ(ptr_scan_mf->l_messageInfo.front().time, 52);
    ASSERT_EQ(call_time_, 0);

    // match so not be storaged
    ptr_scan_mf->handleMessage(51.0, ptr_message);
    ASSERT_EQ(ptr_scan_mf->l_messageInfo.size(), 50);
    ASSERT_DOUBLE_EQ(ptr_scan_mf->l_messageInfo.front().time, 52);
    ASSERT_EQ(call_time_, 1);
  }
  unsigned int call_time_ = 0;
};

TEST(MessageFilterTester, handleTransformableMsgTest) {
  MessageFilterTester messageFilterTester;
  messageFilterTester.handleTransformableMsgTest();
}

TEST(MessageFilterTester, inputTransformTest) {
  MessageFilterTester messageFilterTester;
  messageFilterTester.inputTransformTest();
}

TEST(MessageFilterTester, getLatestTransformTest) {
  MessageFilterTester messageFilterTester;
  messageFilterTester.getLatestTransformTest();
}

TEST(MessageFilterTester, handleMessageTest) {
  MessageFilterTester messageFilterTester;
  messageFilterTester.handleMessageTest();
}

}  // namespace CVTE_BABOT

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// transformLaserScanToPoint(ptr_scan_msg);
// TimeType time = tf2_ros::timeToSec(ptr_scan_msg->header.stamp);

// // there a bug that use back() is incorrect if multisensor.
// this->v_laser_scan_message_filter_.back()->inputMessage(time, ptr_scan_msg);
//
