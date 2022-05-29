#include "costmap_params.hpp"
#include <gtest/gtest.h>

namespace CVTE_BABOT {
class CostmapParamsTester : public testing::Test {
public:
  // 测试设置参数函数和获取参数函数是否正常
  void testSetParams() {
    CostmapParameters params;

    // int
    int int_params = 0;
    EXPECT_FALSE(params.getParam<int>("abc", int_params));
    EXPECT_EQ(int_params, 0);

    params.setParam<int>("abc", 10);
    EXPECT_TRUE(params.getParam<int>("abc", int_params));
    EXPECT_EQ(int_params, 10);

    int int_params2 = 0;
    EXPECT_FALSE(params.getParam<int>("abcd", int_params2));
    EXPECT_EQ(int_params2, 0);

    int int_params3 = 0;
    params.setParam<int>("double", 10.0);
    EXPECT_TRUE(params.getParam<int>("double", int_params3)); // ??
    EXPECT_EQ(int_params3, 10);

    // bool
    bool bool_p = false;
    EXPECT_FALSE(params.getParam<bool>("abc", bool_p));
    EXPECT_FALSE(bool_p);

    params.setParam<bool>("abc", true);
    EXPECT_TRUE(params.getParam<bool>("abc", bool_p));
    EXPECT_TRUE(bool_p);

    // double
    double double_p = 0;
    EXPECT_FALSE(params.getParam<double>("abc", double_p));
    ASSERT_NEAR(double_p, 0, 1e-6);

    double double_p2 = 0;
    params.setParam<double>("abc", 54.568);
    EXPECT_TRUE(params.getParam<double>("abc", double_p2));
    ASSERT_NEAR(double_p2, 54.568, 1e-6);

    // string
    std::string string_p = "";
    EXPECT_FALSE(params.getParam<std::string>("abc", string_p));
    EXPECT_EQ(string_p, "");

    std::string string_p2 = "";
    params.setParam<std::string>("abc", "abc");
    EXPECT_TRUE(params.getParam<std::string>("abc", string_p2));
    EXPECT_EQ(string_p2, "abc");
  }

protected:
  virtual void TestBody() {}
  virtual void SetUp() { //初始化函数
  }

  virtual void TearDown() { //清理函数
  }
};
CostmapParamsTester params_test;

TEST_F(CostmapParamsTester, testSetParams) { params_test.testSetParams(); }

} // namespace CVTE_BABOT

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}