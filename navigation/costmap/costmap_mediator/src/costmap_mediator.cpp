#include "costmap_mediator.hpp"
#include "costmap_2d.hpp"

namespace CVTE_BABOT {
std::shared_ptr<CostmapMediator> CostmapMediator::ptr_instance_ = nullptr;
std::map<std::string, std::mutex> CostmapMediator::m_lock_;

CostmapMediator::CostmapMediator() { ptr_costmap_params_ = nullptr; }

std::shared_ptr<CostmapMediator> CostmapMediator::getPtrInstance() {
  if (!ptr_instance_.get()) {
    ptr_instance_.reset(new CostmapMediator());
    /// add code here
    /*to read param from yaml or other type files*/
  }
  return ptr_instance_;
}

std::shared_ptr<const Costmap2d> CostmapMediator::getCostmap() {
  return ptr_layered_costmap_->getCostmap();
}

const std::vector<CostmapPoint> &CostmapMediator::getFootprint() {
  return footprint_;
}

bool CostmapMediator::findDynamicCar(WorldmapPoint &dynamic_car_point) {
  std::lock_guard<std::mutex> lock(dynamic_mutex_);
  if (b_find_dynamic_cars_) {
    dynamic_car_point = avoidance_car_position_;
    // 每次查询状态后清除标志
    b_find_dynamic_cars_ = false;
    return true;
  } else {
    return false;
  }
}
}  // namespace CVTE_BABOT