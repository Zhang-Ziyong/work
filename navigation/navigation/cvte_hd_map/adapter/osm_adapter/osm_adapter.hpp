/**
 * @file osm_adapter.hpp
 * @author linyanlong(linyanlong@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-12-23
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef OSM_ADAPTER_HPP_
#define OSM_ADAPTER_HPP_
#include "math_utils.h"
#include <string>
#include "map.pb.h"
#include <map>

namespace cvte {
namespace hdmap {
enum AreaType {
  slope,
  narrow_aisle,
  speed_hump,
  bussiness,
  elevator,
  black,
};
class OsmAdapter {
 public:
  OsmAdapter();
  /**
   * @brief 加载osm数据文件
   *
   * @param filename
   * @param pb_map
   * @return true
   * @return false
   */
  bool static LoadData(const std::string &filename, Map *pb_map);

 private:
  template <class AttributeT, class AreaT>
  void static setAreaAttribute(const AttributeT &attribute, AreaT &region);

  std::vector<math_utils::Vec2d> static generaRect(
      const math_utils::Vec2d &point1, const math_utils::Vec2d &point2);
};
}  // namespace hdmap
}  // namespace cvte

#endif