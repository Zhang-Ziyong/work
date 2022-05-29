/*
* brief : test_local_planner_factory.cpp 文件的单元测试
*
* create: 2019-12-17
*
* tester: liangjiajun
*/


#ifndef TEST_LOCAL_PLANNER_FACTORY
#define TEST_LOCAL_PLANNER_FACTORY

#include "local_planner_factory.hpp"

#include <gtest/gtest.h>
#include <iostream>
#include <string>

namespace CVTE_BABOT {

class LCPlannerFactoryTest : public testing::Test {
public:
    virtual void TestBody() {}
    LCPlannerFactoryTest();

    void testCreatePlanner();

private:
    std::shared_ptr<LCPlannerFactory> lc_planner_factory_ptr_;

};

LCPlannerFactoryTest::LCPlannerFactoryTest() {
    lc_planner_factory_ptr_ = std::make_shared<LCPlannerFactory>();
}

void LCPlannerFactoryTest::testCreatePlanner(){
    auto ptr_1 = lc_planner_factory_ptr_->createLocalPlanner("dijkstra", 100, 100);
    EXPECT_NE(nullptr, ptr_1);

    auto ptr_2 = lc_planner_factory_ptr_->createLocalPlanner("hybrid", 100, 100);
    EXPECT_NE(nullptr, ptr_2);

    auto ptr_3 = lc_planner_factory_ptr_->createLocalPlanner("aaaaaa", 100, 100);
    EXPECT_NE(nullptr, ptr_3);
}

}

CVTE_BABOT::LCPlannerFactoryTest *lc_planner_factory_tester = NULL;

TEST(LCPlannerFactoryTest, testCreatePlanner) { lc_planner_factory_tester->testCreatePlanner(); }

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
    lc_planner_factory_tester = new CVTE_BABOT::LCPlannerFactoryTest();
  return RUN_ALL_TESTS();
}

#endif // end of TEST_LOCAL_PLANNER_FACTORY