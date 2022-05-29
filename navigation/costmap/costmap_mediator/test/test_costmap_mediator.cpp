/************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2018, CVTE.
* All rights reserved.
*
*@file test_costmap_mediator.cpp
*
*@brief test_costmap_mediator.cpp 文件

*@author caojun <caojun@cvte.com>
*
*@modified yangfangping <yangfangping@cvte.com>
*@version current_algo.dev.1.0
*@data 2019-04-18
************************************************************************/

#include <gtest/gtest.h>
#include <vector>
#include "costmap_mediator.hpp"
namespace CVTE_BABOT {
typedef struct {
  double test_x = 0.0;
  double test_y = 0.0;
} data_temp;

typedef struct {
  int a = 0;
  int b = 0;
} data_temp2;

class CostmapMediatorTester : public testing::Test {
 public:
  // 测试设置参数函数和获取参数函数是否正常
  // TODO:
  // 风险：在调用的时候，如果没有设置对应值会在存储中初始化一个对应参数名的默认参数，在get时并不能知道获得的参数是否有效；
  void testData() {
    auto cm = CostmapMediator::getPtrInstance();
    cm->registerDataType<int>("int", 1);
    int data_int = 0;
    cm->getData("int", data_int);
    EXPECT_EQ(data_int, 1);

    data_int = 0;

    cm->getData("int2", data_int);
    EXPECT_EQ(data_int, 0);

    double data_double = 1.0;
    cm->getData("int", data_double);
    EXPECT_EQ(data_double, 0.0);

    data_temp2 test2;
    test2.a = 3;
    test2.b = 4;
    std::vector<data_temp2> data_buffer2;
    data_buffer2.reserve(10);
    data_buffer2.push_back(test2);
    std::shared_ptr<std::vector<data_temp2>> ptr_data_buffer2 =
        std::make_shared<std::vector<data_temp2>>(data_buffer2);
    auto ptr_data_buffer3 = ptr_data_buffer2;
    cm->getData(std::string("data_temp3"), ptr_data_buffer2);
    // std::cout << "size of temp vector" << (*test_data_temp3).size()
    //         << std::endl;
    EXPECT_EQ(ptr_data_buffer2.get(), ptr_data_buffer3.get());
    // EXPECT_EQ((*test_data_temp3).size(), 2);
    // EXPECT_EQ((*test_data_temp3)[1].a, 3);
    // EXPECT_EQ((*test_data_temp3)[1].b, 4);
  }

  /**
   * testCostmapParameters
   * @brief 测试设置和获取参数是否正确
   * */
  void testCostmapParameters() {
    auto ptr_cp = std::make_shared<CostmapParameters>();
    auto cm = CostmapMediator::getPtrInstance();
    EXPECT_FALSE(cm->isParameterReady());
    cm->setCostmapParameters(ptr_cp);
    EXPECT_TRUE(cm->isParameterReady());
    // int
    int int_params = 0;
    int int_default = 0;
    cm->getParam<int>("abc", int_params, int_default);
    EXPECT_EQ(int_params, int_default);

    ptr_cp->setParam<int>("abc", 10);
    cm->getParam<int>("abc", int_params, int_default);
    EXPECT_EQ(int_params, 10);

    int int_params2 = 0;
    cm->getParam<int>("abcd", int_params2, int_default);
    EXPECT_EQ(int_params2, int_default);

    int int_params3 = 0;
    ptr_cp->setParam<int>("double", 10.0);
    cm->getParam<int>("double", int_params3, int_default);  // ??
    EXPECT_EQ(int_params3, 10);

    // bool
    bool bool_p = false;
    bool default_bool = false;
    cm->getParam<bool>("abc", bool_p, default_bool);
    EXPECT_FALSE(bool_p);

    ptr_cp->setParam<bool>("abc", true);
    cm->getParam<bool>("abc", bool_p, default_bool);
    EXPECT_TRUE(bool_p);

    // double
    double double_p = 0;
    double default_double = 1231.2121;
    cm->getParam<double>("abc", double_p, default_double);
    ASSERT_NEAR(double_p, default_double, 1e-6);

    double double_p2 = 0;
    ptr_cp->setParam<double>("abc", 54.568);
    cm->getParam<double>("abc", double_p2, default_double);
    ASSERT_NEAR(double_p2, 54.568, 1e-6);

    // string
    std::string string_p = "";
    std::string default_string = "ssssss";
    cm->getParam<std::string>("abc", string_p, default_string);
    EXPECT_EQ(string_p, default_string);

    std::string string_p2 = "";
    ptr_cp->setParam<std::string>("abc", "abc");
    cm->getParam<std::string>("abc", string_p2, default_string);
    EXPECT_EQ(string_p2, "abc");
  }

  /**
   * tesLayeredCostmap
   * @brief 测试costmap设置是否正确
   * */

  void tesLayeredCostmap() {
    auto cm = CostmapMediator::getPtrInstance();
    EXPECT_FALSE(cm->isLayeredCostmapReady());

    auto lc = std::make_shared<LayeredCostmap>(true, true);
    cm->setLayeredCostmap(lc);
    EXPECT_TRUE(cm->isLayeredCostmapReady());
    EXPECT_TRUE(cm->isRolling());
  }

  /**
   * testRegisterUpdateFunc
   * @brief 测试registerUpdateFunc函数,updateData, getData函数，是否运行正确
   * 1.注册两个函数和数据，看函数列表中是否能找到，值是否正确
   * 2.测试updata
   * 3.异常检测，测试没注册的数据和函数
   * */
  void testRegisterUpdateFunc() {
    auto cm = CostmapMediator::getPtrInstance();  // 构造实例
    data_temp test;
    test.test_x = 15.0;
    test.test_y = 17.0;
    std::vector<data_temp> data_buffer;
    data_buffer.reserve(10);
    data_buffer.push_back(test);
    typedef std::shared_ptr<std::vector<data_temp>> StorageTypeTmp1;
    StorageTypeTmp1 ptr_data_buffer =
        std::make_shared<std::vector<data_temp>>(data_buffer);

    cm->registerUpdateFunc<StorageTypeTmp1>(std::string("data_temp"),
                                            ptr_data_buffer);

    data_temp2 test2;
    test2.a = 3;
    test2.b = 4;
    std::vector<data_temp2> data_buffer2;
    data_buffer2.reserve(10);
    data_buffer2.push_back(test2);

    typedef std::shared_ptr<std::vector<data_temp2>> StorageTypeTmp2;
    StorageTypeTmp2 ptr_data_buffer2 =
        std::make_shared<std::vector<data_temp2>>(data_buffer2);
    // std::cout << (*ptr_data_buffer2)[0].a << std::endl;

    cm->registerUpdateFunc<StorageTypeTmp2>(std::string("data_temp2"),
                                            ptr_data_buffer2);
    std::shared_ptr<std::vector<data_temp2>> test_use =
        cm->m_data_<std::shared_ptr<std::vector<data_temp2>>>[std::string(
            "data_temp2")];
    std::shared_ptr<std::vector<data_temp>> test3 =
        cm->m_data_<std::shared_ptr<std::vector<data_temp>>>[std::string(
            "data_temp")];
    EXPECT_EQ((*test3)[0].test_x, test.test_x);
    EXPECT_EQ((*test3)[0].test_y, test.test_y);
    EXPECT_EQ((*test_use)[0].a, test2.a);
    EXPECT_EQ((*test_use)[0].b, test2.b);
    // std::cout << (*test3)[0].test_x << std::endl;

    data_temp2 test_func;
    test_func.a = 3;
    test_func.b = 4;
    ptr_data_buffer2->push_back(test_func);

    cm->updateData(std::string("data_temp2"), ptr_data_buffer2);
    EXPECT_EQ((*test_use).size(), 2);
    EXPECT_EQ((*test_use)[1].a, 3);
    EXPECT_EQ((*test_use)[1].b, 4);

    data_temp test_func1;
    test_func1.test_x = 25.0;
    test_func1.test_y = 27.0;
    ptr_data_buffer->push_back(test_func1);

    cm->updateData(std::string("data_temp"), ptr_data_buffer);
    EXPECT_EQ((*test3).size(), 2);
    EXPECT_EQ((*test3)[1].test_x, 25.0);
    EXPECT_EQ((*test3)[1].test_y, 27.0);

    /// 测试getData
    std::shared_ptr<std::vector<data_temp>> test_data_temp;
    cm->getData(std::string("data_temp"), test_data_temp);
    EXPECT_EQ(test_data_temp.get(), test3.get());
    EXPECT_EQ((*test_data_temp).size(), 2);
    EXPECT_EQ((*test_data_temp)[1].test_x, 25.0);
    EXPECT_EQ((*test_data_temp)[1].test_y, 27.0);

    std::shared_ptr<std::vector<data_temp2>> test_data_temp2;
    cm->getData(std::string("data_temp2"), test_data_temp2);
    EXPECT_EQ(test_data_temp2.get(), test_use.get());
    EXPECT_EQ((*test_data_temp2).size(), 2);
    EXPECT_EQ((*test_data_temp2)[1].a, 3);
    EXPECT_EQ((*test_data_temp2)[1].b, 4);

    /// 异常测试
    cm->updateData(std::string("data_temp3"), ptr_data_buffer);
  }

 protected:
  virtual void TestBody() {}
  virtual void SetUp() {  //初始化函数
  }

  virtual void TearDown() {  //清理函数
  }
};

CostmapMediatorTester mediator_test;

TEST_F(CostmapMediatorTester, testData) { mediator_test.testData(); }

TEST_F(CostmapMediatorTester, testCostmapParameters) {
  mediator_test.testCostmapParameters();
}
TEST_F(CostmapMediatorTester, tesLayeredCostmap) {
  mediator_test.tesLayeredCostmap();
}

TEST_F(CostmapMediatorTester, testRegisterUpdateFunc) {
  mediator_test.testRegisterUpdateFunc();
}
}  // namespace CVTE_BABOT

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}