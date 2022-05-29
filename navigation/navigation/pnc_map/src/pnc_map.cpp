#include "pnc_map.hpp"

namespace CVTE_BABOT {

bool PncMap::InPitArea(const math_utils::Vec2d &point) const {
  auto ptr_map_area =
      ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
  if (ptr_map_area == nullptr) {
    return false;
  }
  cvte::hdmap::Id id;
  return ptr_map_area->inPitArea(point, id);
}

bool PncMap::InSlopeArea(const math_utils::Vec2d &point) const {
  auto ptr_map_area =
      ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
  if (ptr_map_area == nullptr) {
    return false;
  }
  cvte::hdmap::Id id;
  return ptr_map_area->inSlopeArea(point, id);
}

bool PncMap::InElevatorArea(const math_utils::Vec2d &point) const {
  auto ptr_map_area =
      ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
  if (ptr_map_area == nullptr) {
    return false;
  }
  cvte::hdmap::Id id;
  return ptr_map_area->inElevatorArea(point, id);
}

bool PncMap::InNarrowArea(const math_utils::Vec2d &point) const {
  auto ptr_map_area =
      ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
  if (ptr_map_area == nullptr) {
    return false;
  }
  cvte::hdmap::Id id;
  return ptr_map_area->inNarrowArea(point, id);
}

// bool PncMap::InCarpetArea(const math_utils::Vec2d &point) const {
//   auto ptr_map_area =
//       ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
//   if (ptr_map_area == nullptr) {
//     return false;
//   }
//   cvte::hdmap::Id id;
//   return ptr_map_area->inCarpetArea(point, id);
// }

bool PncMap::InCleanArea(const math_utils::Vec2d &point) const {
  auto ptr_map_area =
      ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
  if (ptr_map_area == nullptr) {
    return false;
  }
  cvte::hdmap::Id id;
  return ptr_map_area->inCleanArea(point, id);
}

bool PncMap::InProhibitedArea(const math_utils::Vec2d &point) const {
  auto ptr_map_area =
      ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
  if (ptr_map_area == nullptr) {
    return false;
  }
  cvte::hdmap::Id id;
  return ptr_map_area->inProhibitedArea(point, id);
}

bool PncMap::InBlackArea(const math_utils::Vec2d &point) const {
  auto ptr_map_area =
      ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
  if (ptr_map_area == nullptr) {
    return false;
  }
  cvte::hdmap::Id id;
  auto slope_areas = ptr_map_area->slopeAreas();
  for (const auto &it : slope_areas) {
    if (it->hasColor() && it->color() == "black") {
      if (it->inSlopeArea(point)) {
        return true;
      }
    }
  }
  auto narrow_areas = ptr_map_area->narrowAreas();
  for (const auto &it : narrow_areas) {
    if (it->hasColor() && it->color() == "black") {
      if (it->inNarrowArea(point)) {
        return true;
      }
    }
  }
  auto bussiness_area = ptr_map_area->cleanAreas();
  for (const auto &it : bussiness_area) {
    if (it->hasColor() && it->color() == "black") {
      if (it->inCleanArea(point)) {
        return true;
      }
    }
  }
  auto speed_bumps = ptr_map_area->speedBumpAreas();
  for (const auto &it : speed_bumps) {
    if (it->hasColor() && it->color() == "black") {
      if (it->inSpeedBump(point)) {
        return true;
      }
    }
  }
  auto mark_areas = ptr_map_area->MarkAreas();
  for (const auto &it : mark_areas) {
    if (it->hasColor() && it->color() == "black") {
      if (it->inMarkArea(point)) {
        return true;
      }
    }
  }
  return false;
}

bool PncMap::ForwardInSlopeArea(int path_index, double distance) const {
  auto ptr_map_area =
      ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
  if (ptr_map_area == nullptr) {
    return false;
  }
  if (ptr_subpath_ == nullptr) {
    return false;
  }

  cvte::hdmap::Id id;
  for (int index = path_index; index < ptr_subpath_->wps.size(); index += 5) {
    double length = ptr_subpath_->wpis[index].path_length -
                    ptr_subpath_->wpis[path_index].path_length;
    if (length > distance) {
      break;
    } else {
      if (InSlopeArea(math_utils::Vec2d(ptr_subpath_->wps[index].getX(),
                                        ptr_subpath_->wps[index].getY()))) {
        LOG(INFO) << "forward in slope: "
                  << ptr_subpath_->wpis[index].path_length -
                         ptr_subpath_->wpis[path_index].path_length;
        return true;
      }
    }
  }
  return false;
}

bool PncMap::ForwardInPitArea(int path_index, double distance) const {
  auto ptr_map_area =
      ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
  if (ptr_map_area == nullptr) {
    return false;
  }
  if (ptr_subpath_ == nullptr) {
    return false;
  }

  cvte::hdmap::Id id;
  for (int index = path_index; index < ptr_subpath_->wps.size(); index += 5) {
    if (ptr_subpath_->wpis[index].path_length -
            ptr_subpath_->wpis[path_index].path_length >
        distance) {
      break;
    } else {
      if (InPitArea(math_utils::Vec2d(ptr_subpath_->wps[index].getX(),
                                      ptr_subpath_->wps[index].getY()))) {
        LOG(INFO) << "forward in pit: "
                  << ptr_subpath_->wpis[index].path_length -
                         ptr_subpath_->wpis[path_index].path_length;
        return true;
      }
    }
  }
  return false;
}

bool PncMap::BackwardInPitArea(int path_index, double distance) const {
  auto ptr_map_area =
      ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
  if (ptr_map_area == nullptr) {
    return false;
  }
  if (ptr_subpath_ == nullptr) {
    return false;
  }

  cvte::hdmap::Id id;
  for (int index = path_index; index >= 5; index -= 5) {
    if (ptr_subpath_->wpis[path_index].path_length -
            ptr_subpath_->wpis[index].path_length >
        distance) {
      break;
    } else {
      if (InPitArea(math_utils::Vec2d(ptr_subpath_->wps[index].getX(),
                                      ptr_subpath_->wps[index].getY()))) {
        LOG(INFO) << "Backward in pit: "
                  << ptr_subpath_->wpis[index].path_length -
                         ptr_subpath_->wpis[path_index].path_length;
        return true;
      }
    }
  }
  return false;
}

bool PncMap::ForwardInElevatorArea(int path_index, double distance) const {
  auto ptr_map_area =
      ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
  if (ptr_map_area == nullptr) {
    return false;
  }
  if (ptr_subpath_ == nullptr) {
    return false;
  }

  cvte::hdmap::Id id;
  for (int index = path_index; index < ptr_subpath_->wps.size(); index += 5) {
    // LOG(INFO) << "index: " << index << " " << ptr_subpath_->wps[index].getX()
    //           << " " << ptr_subpath_->wps[index].getY()
    //           << " length: " << ptr_subpath_->wpis[index].path_length << " "
    //           << ptr_subpath_->wpis[path_index].path_length;
    double length = ptr_subpath_->wpis[index].path_length -
                    ptr_subpath_->wpis[path_index].path_length;
    if (length > distance) {
      break;
    } else {
      if (InElevatorArea(math_utils::Vec2d(ptr_subpath_->wps[index].getX(),
                                           ptr_subpath_->wps[index].getY()))) {
        LOG(INFO) << "forward in elevator: "
                  << ptr_subpath_->wpis[index].path_length -
                         ptr_subpath_->wpis[path_index].path_length;
        return true;
      }
    }
  }
  return false;
}

bool PncMap::ForwardInNarrowArea(int path_index, double distance) const {
  auto ptr_map_area =
      ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
  if (ptr_map_area == nullptr) {
    return false;
  }
  if (ptr_subpath_ == nullptr) {
    return false;
  }

  cvte::hdmap::Id id;
  for (int index = path_index; index < ptr_subpath_->wps.size(); index += 5) {
    double length = ptr_subpath_->wpis[index].path_length -
                    ptr_subpath_->wpis[path_index].path_length;
    if (length > distance) {
      break;
    } else {
      if (InNarrowArea(math_utils::Vec2d(ptr_subpath_->wps[index].getX(),
                                         ptr_subpath_->wps[index].getY()))) {
        LOG(INFO) << "forward in narrow: "
                  << ptr_subpath_->wpis[index].path_length -
                         ptr_subpath_->wpis[path_index].path_length;
        return true;
      }
    }
  }
  return false;
}

// bool PncMap::ForwardInCarpetArea(int path_index, double distance) const {
//   auto ptr_map_area =
//       ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
//   if (ptr_map_area == nullptr) {
//     return false;
//   }
//   if (ptr_subpath_ == nullptr) {
//     return false;
//   }
//   int index = getForwardPathIndex(path_index, distance);
//   if (index >= ptr_subpath_->wps.size()) {
//     return false;
//   }
//   cvte::hdmap::Id id;
//   return ptr_map_area->inCarpetArea(
//       math_utils::Vec2d(ptr_subpath_->wps[index].getX(),
//                         ptr_subpath_->wps[index].getY()),
//       id);
// }

bool PncMap::ForwardInCleanArea(int path_index, double distance) const {
  auto ptr_map_area =
      ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
  if (ptr_map_area == nullptr) {
    return false;
  }
  if (ptr_subpath_ == nullptr) {
    return false;
  }

  cvte::hdmap::Id id;
  for (int index = path_index; index < ptr_subpath_->wps.size(); index += 5) {
    double length = ptr_subpath_->wpis[index].path_length -
                    ptr_subpath_->wpis[path_index].path_length;
    if (length > distance) {
      break;
    } else {
      if (InCleanArea(math_utils::Vec2d(ptr_subpath_->wps[index].getX(),
                                        ptr_subpath_->wps[index].getY()))) {
        return true;
      }
    }
  }
  return false;
}

bool PncMap::ForwardInProhibitedArea(int path_index, double distance) const {
  auto ptr_map_area =
      ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
  if (ptr_map_area == nullptr) {
    return false;
  }
  if (ptr_subpath_ == nullptr) {
    return false;
  }

  cvte::hdmap::Id id;
  for (int index = path_index; index < ptr_subpath_->wps.size(); index += 5) {
    double length = ptr_subpath_->wpis[index].path_length -
                    ptr_subpath_->wpis[path_index].path_length;
    if (length > distance) {
      break;
    } else {
      if (InProhibitedArea(
              math_utils::Vec2d(ptr_subpath_->wps[index].getX(),
                                ptr_subpath_->wps[index].getY()))) {
        LOG(INFO) << "forward in prohibited: "
                  << ptr_subpath_->wpis[index].path_length -
                         ptr_subpath_->wpis[path_index].path_length;
        return true;
      }
    }
  }
  return false;
}

bool PncMap::ForwardInBlackArea(int path_index, double distance) const {
  auto ptr_map_area =
      ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
  if (ptr_map_area == nullptr) {
    return false;
  }
  if (ptr_subpath_ == nullptr) {
    return false;
  }

  cvte::hdmap::Id id;
  for (int index = path_index; index < ptr_subpath_->wps.size(); index += 5) {
    double length = ptr_subpath_->wpis[index].path_length -
                    ptr_subpath_->wpis[path_index].path_length;
    if (length > distance) {
      break;
    } else {
      if (InBlackArea(math_utils::Vec2d(ptr_subpath_->wps[index].getX(),
                                        ptr_subpath_->wps[index].getY()))) {
        LOG(INFO) << "forward in black area: "
                  << ptr_subpath_->wpis[index].path_length -
                         ptr_subpath_->wpis[path_index].path_length;
        return true;
      }
    }
  }
  return false;
}

std::vector<cvte::hdmap::Polygon2d> PncMap::GetClearArea() const {
  std::vector<cvte::hdmap::Polygon2d> elevator_polygons = GetElevatorArea();
  std::vector<cvte::hdmap::Polygon2d> narrow_polygons = GetNarrowArea();
  std::vector<cvte::hdmap::Polygon2d> clear_polygons;
  clear_polygons.reserve(elevator_polygons.size() + narrow_polygons.size());
  for (const auto &it : elevator_polygons) { clear_polygons.push_back(it); }
  for (const auto &it : narrow_polygons) { clear_polygons.push_back(it); }
  return clear_polygons;
}

std::vector<cvte::hdmap::Polygon2d> PncMap::GetCleanArea() const {
  auto ptr_map_area =
      ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
  if (ptr_map_area == nullptr) {
    return std::vector<cvte::hdmap::Polygon2d>();
  }
  auto clean_areas = ptr_map_area->cleanAreas();
  std::vector<cvte::hdmap::Polygon2d> polygons;
  polygons.reserve(clean_areas.size());
  for (const auto &it : clean_areas) { polygons.push_back(it->polygon()); }
  return polygons;
}

// std::vector<cvte::hdmap::Polygon2d> PncMap::GetCarpetArea() const {
//   auto ptr_map_area =
//       ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
//   if (ptr_map_area == nullptr) {
//     return std::vector<cvte::hdmap::Polygon2d>();
//   }
//   auto carpet_areas = ptr_map_area->carpetAreas();
//   std::vector<cvte::hdmap::Polygon2d> polygons;
//   polygons.reserve(carpet_areas.size());
//   for (const auto &it : carpet_areas) { polygons.push_back(it->polygon()); }
//   return polygons;
// }

std::vector<cvte::hdmap::Polygon2d> PncMap::GetPitArea() const {
  auto ptr_map_area =
      ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
  if (ptr_map_area == nullptr) {
    return std::vector<cvte::hdmap::Polygon2d>();
  }
  auto pit_areas = ptr_map_area->pitAreas();
  std::vector<cvte::hdmap::Polygon2d> polygons;
  polygons.reserve(pit_areas.size());
  for (const auto &it : pit_areas) { polygons.push_back(it->polygon()); }
  return polygons;
}

std::vector<cvte::hdmap::Polygon2d> PncMap::GetSlopeArea() const {
  auto ptr_map_area =
      ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
  if (ptr_map_area == nullptr) {
    return std::vector<cvte::hdmap::Polygon2d>();
  }
  auto slope_areas = ptr_map_area->slopeAreas();
  std::vector<cvte::hdmap::Polygon2d> polygons;
  polygons.reserve(slope_areas.size());
  for (const auto &it : slope_areas) { polygons.push_back(it->polygon()); }
  return polygons;
}

std::vector<cvte::hdmap::Polygon2d> PncMap::GetElevatorArea() const {
  auto ptr_map_area =
      ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
  if (ptr_map_area == nullptr) {
    return std::vector<cvte::hdmap::Polygon2d>();
  }
  auto elevator_areas = ptr_map_area->elecatorAreas();
  std::vector<cvte::hdmap::Polygon2d> polygons;
  polygons.reserve(elevator_areas.size());
  for (const auto &it : elevator_areas) { polygons.push_back(it->polygon()); }
  return polygons;
}

std::vector<cvte::hdmap::Polygon2d> PncMap::GetProhibitedArea() const {
  auto ptr_map_area =
      ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
  if (ptr_map_area == nullptr) {
    return std::vector<cvte::hdmap::Polygon2d>();
  }
  auto prohibite_areas = ptr_map_area->prohibitedAreas();
  std::vector<cvte::hdmap::Polygon2d> polygons;
  polygons.reserve(prohibite_areas.size());
  for (const auto &it : prohibite_areas) { polygons.push_back(it->polygon()); }
  auto bussiness_areas = ptr_map_area->cleanAreas();
  for (const auto &it : bussiness_areas) {
    if (it->hasMaterial() && it->material() == "carpet") {
      polygons.push_back(it->polygon());
    }
  }
  auto narrow_areas = ptr_map_area->narrowAreas();
  for (const auto &it : narrow_areas) {
    if (it->hasMaterial() && it->material() == "carpet") {
      polygons.push_back(it->polygon());
    }
  }
  auto slope_areas = ptr_map_area->slopeAreas();
  for (const auto &it : slope_areas) {
    if (it->hasMaterial() && it->material() == "carpet") {
      polygons.push_back(it->polygon());
    }
  }
  auto speed_humps = ptr_map_area->speedBumpAreas();
  for (const auto &it : speed_humps) {
    if (it->hasMaterial() && it->material() == "carpet") {
      polygons.push_back(it->polygon());
    }
  }
  auto mark_areas = ptr_map_area->MarkAreas();
  for (const auto &it : mark_areas) {
    if (it->hasMaterial() && it->material() == "carpet") {
      polygons.push_back(it->polygon());
    }
  }
  return polygons;
}

std::vector<cvte::hdmap::Polygon2d> PncMap::GetNarrowArea() const {
  auto ptr_map_area =
      ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
  if (ptr_map_area == nullptr) {
    return std::vector<cvte::hdmap::Polygon2d>();
  }
  auto narrow_areas = ptr_map_area->narrowAreas();
  std::vector<cvte::hdmap::Polygon2d> polygons;
  polygons.reserve(narrow_areas.size());
  for (const auto &it : narrow_areas) { polygons.push_back(it->polygon()); }
  return polygons;
}

bool PncMap::GetPitBox(const math_utils::Vec2d &point,
                       cvte::hdmap::Box2d &box) const {
  auto ptr_map_area =
      ptr_cvte_hdmap_->GetMapAreaById(ptr_cvte_hdmap_->GetCurrentMapId());
  if (ptr_map_area == nullptr) {
    return false;
  }
  return ptr_map_area->getPitBox(point, box);
}

int PncMap::getForwardPathIndex(int path_index, double distance) const {
  int index = path_index;
  for (; index < ptr_subpath_->wps.size(); index++) {
    if (ptr_subpath_->wpis[index].path_length -
            ptr_subpath_->wpis[path_index].path_length >
        distance) {
      break;
    }
  }
  return index - 1;
}

}  // namespace CVTE_BABOT