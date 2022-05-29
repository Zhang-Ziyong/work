#include "costmap2d_builder.hpp"
#include "costmap2d_director.hpp"
#include <iostream>
#include <layered_costmap.hpp>

int main() {
  bool rolling_window, b_track_unknown_space;
  rolling_window = true;
  b_track_unknown_space = false;
  auto layered_costmap = std::make_shared<CVTE_BABOT::LayeredCostmap>(
      rolling_window, b_track_unknown_space);

  auto builder =
      std::make_shared<CVTE_BABOT::Costmap2dBuilder>(layered_costmap);

  auto director = CVTE_BABOT::Costmap2dDirector(builder);
  CVTE_BABOT::LayerParammeter lp;

  lp.addLayer(CVTE_BABOT::StaticLayerType, "global_costmap.static_layer");
  lp.addLayer(CVTE_BABOT::StaticLayerType, "global_costmap.static_layer");
  lp.addLayer(CVTE_BABOT::ObstacleLayerType, "global_costmap.obstacle_layer");
  lp.addLayer(CVTE_BABOT::RangeSensorLayerType,
              "global_costmap.range_sensor_layer");
  lp.addLayer(CVTE_BABOT::InflationLayerType, "global_costmap.inflation_layer");
  director.buildCostmap(lp);
  while (1) {}

  return 1;
}