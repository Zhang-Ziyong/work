/*
* brief : global_planner_factory.cpp 文件的单元测试
*
* create: 2019-12-17
*
* tester: liangjiajun
*/


#ifndef TEST_GLOBAL_PLANNER_FACTORY
#define TEST_GLOBAL_PLANNER_FACTORY

#include "global_planner_factory.hpp"
#include "global_planner.hpp"


#include <gtest/gtest.h>
#include <iostream>
#include <string>

namespace CVTE_BABOT {

class GBPlannerFactoryTest : public testing::Test {
public:
    virtual void TestBody() {}
    GBPlannerFactoryTest();

    void testCreatePlanner();

private:
    std::shared_ptr<GBPlannerFactory> gb_planner_factory_ptr_;

};

GBPlannerFactoryTest::GBPlannerFactoryTest() {
    gb_planner_factory_ptr_ = std::make_shared<GBPlannerFactory>();
}

void GBPlannerFactoryTest::testCreatePlanner(){
    std::shared_ptr<GlobalPlanner> ptr_1 = gb_planner_factory_ptr_->createGlobalPlanner("dijkstra", 100, 100);
    EXPECT_NE(nullptr, ptr_1);

    auto ptr_2 = gb_planner_factory_ptr_->createGlobalPlanner("hybrid", 100, 100);
    EXPECT_NE(nullptr, ptr_2);

    auto ptr_3 = gb_planner_factory_ptr_->createGlobalPlanner("aaaaaa", 100, 100);
    EXPECT_NE(nullptr, ptr_3);
}

}

CVTE_BABOT::GBPlannerFactoryTest *gb_planner_factory_tester = NULL;

TEST(GBPlannerFactoryTest, testCreatePlanner) { gb_planner_factory_tester->testCreatePlanner(); }

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
    gb_planner_factory_tester = new CVTE_BABOT::GBPlannerFactoryTest();
  return RUN_ALL_TESTS();
}

#endif // end of TEST_GLOBAL_PLANNER_FACTORY