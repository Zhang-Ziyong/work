/**
 * @file osm_adapter_test.cpp
 * @author linyanlong(linyanlong@cvte.com)
 * @brief
 * @version 0.1
 * @date 2021-12-30
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "osm_adapter.hpp"
#include <string>
#include "gtest/gtest.h"

namespace cvte {
namespace hdmap {

class OsmAdapterTestSuite : public ::testing::Test {
 protected:
  OsmAdapterTestSuite(){};
  virtual ~OsmAdapterTestSuite(){};
  virtual void SetUp(){};
  virtual void TearDown(){};

  void InitOsmAdapter(const std::string &osm_file, Map *pb_map);
};

void OsmAdapterTestSuite::InitOsmAdapter(const std::string &osm_file,
                                         Map *pb_map) {
  OsmAdapter osm_adapter;
  osm_adapter.LoadData(osm_file, pb_map);
}

TEST_F(OsmAdapterTestSuite, TestReadOsm) {
  Map pb_map;
  EXPECT_EQ(
      true,
      OsmAdapter::LoadData(
          "src/navigation/navigation/cvte_hd_map/test_data/cvte_lobby.osm",
          &pb_map));
  EXPECT_EQ(false, OsmAdapter::LoadData("test_data/1.osm", &pb_map));
}

TEST_F(OsmAdapterTestSuite, TestOsmData) {
  Map pb_map;
  // std::string map_file =
  // "/home/cvte/Development/CVTERobot/src/navigation/navigation/cvte_hd_map/test_data/cvte_lobby.osm";
  std::string map_file =
      "src/navigation/navigation/cvte_hd_map/test_data/1.osm";
  OsmAdapter::LoadData(map_file, &pb_map);
  EXPECT_EQ(1, pb_map.map_area_size());
  EXPECT_EQ(map_file, pb_map.map_area().begin()->id().id());
  EXPECT_EQ(2, pb_map.clean_area_size());
  EXPECT_EQ(0, pb_map.junction_size());
  EXPECT_EQ(1, pb_map.elevator_size());
  EXPECT_EQ(2, pb_map.mark_area_size());
  EXPECT_EQ(1, pb_map.slope_area_size());
  EXPECT_EQ(2, pb_map.narrow_area_size());
}

}  // namespace hdmap
}  // namespace cvte

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}