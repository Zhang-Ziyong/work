#include "world_map_data.hpp"
#include <gtest/gtest.h>

namespace CVTE_BABOT {
class WorldMapDataTester : public testing::Test {
public:
  // 测试设置参数函数和获取参数函数是否正常
  void test_readMapFromYaml() {
    WorldmapData wm;
    bool read_flag = wm.readMapFromYaml("src/navigation_algo/costmap/"
                                        "costmap_base/test/B1012_2.yaml");
    ASSERT_EQ(read_flag, true);
  }

protected:
  virtual void TestBody() {}
};
WorldMapDataTester world_map_data_test;

TEST_F(WorldMapDataTester, test_readMapFromYaml) {
  world_map_data_test.test_readMapFromYaml();
}

} // namespace CVTE_BABOT

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
