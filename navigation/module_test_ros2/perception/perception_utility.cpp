#include <rclcpp/rclcpp.hpp>
#include "perception_utility.hpp"

void generateCostmapMsg(
    const std::shared_ptr<const CVTE_BABOT::Costmap2d> &ptr_costmap,
    nav_msgs::msg::OccupancyGrid &grid_map) {
  std::array<char, 256> cost_translation_table;
  cost_translation_table[0] = 0;      // NO obstacle
  cost_translation_table[253] = 99;   // INSCRIBED obstacle
  cost_translation_table[254] = 100;  // LETHAL obstacle
  cost_translation_table[255] = -1;   // UNKNOWN
  for (int i = 1; i < 253; i++) {
    cost_translation_table[i] = static_cast<char>(1 + (97 * (i - 1)) / 251);
  }

  grid_map.header.frame_id = "map";
  grid_map.header.stamp = rclcpp::Clock().now();

  double d_resolution = ptr_costmap->getResolution();
  grid_map.info.resolution = d_resolution;
  grid_map.info.width = ptr_costmap->getSizeInCellsX();
  grid_map.info.height = ptr_costmap->getSizeInCellsY();

  CVTE_BABOT::CostmapPoint cm_point;
  cm_point.ui_x = 0;
  cm_point.ui_y = 0;
  CVTE_BABOT::WorldmapPoint wc_point;
  ptr_costmap->mapToWorld(cm_point, wc_point);
  grid_map.info.origin.position.x = wc_point.d_x - d_resolution / 2;
  grid_map.info.origin.position.y = wc_point.d_y - d_resolution / 2;
  grid_map.info.origin.position.z = 0.0;
  grid_map.info.origin.orientation.w = 1.0;

  grid_map.data.resize(grid_map.info.width * grid_map.info.height);
  auto ptr_data = ptr_costmap->getCharMap();
  for (size_t i = 0; i < grid_map.data.size(); i++) {
    grid_map.data[i] = cost_translation_table[ptr_data[i]];
  }
}
