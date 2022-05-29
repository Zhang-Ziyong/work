#ifndef __RANGE_SENSOR_LAYER2_HPP
#define __RANGE_SENSOR_LAYER2_HPP

#include <time.h>

#include <list>
#include <mutex>

#include "costmap_layer.hpp"
namespace CVTE_BABOT {

struct RangeData {
  time_t update_time;
  double x;
  double y;
};

class RangeSensorLayer2 final : public CostmapLayer {
 public:
  RangeSensorLayer2() = default;
  ~RangeSensorLayer2() = default;
  RangeSensorLayer2(const RangeSensorLayer2& obj) = delete;
  RangeSensorLayer2& operator=(const RangeSensorLayer2& obj) = delete;

  bool onInitialize();

  void matchSize() override;

  void getParams() override;

  bool updateBounds(const WorldmapPose& wp_robot_pose,
                    CostmapBound& cb_costmap_bound) override;

  bool updateCosts(const std::shared_ptr<Costmap2d> master_grid,
                   const int& i_min_i, const int& i_min_j, const int& i_max_i,
                   const int& i_max_j) override;

  void updateOrigin(const double& d_new_origin_x, const double& d_new_origin_y);

  void activate() override;
  void deactivate() override;
  void reset() override;

 private:
  static inline double normalizeAngle(const double& angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  std::vector<std::string> range_data_buffer_names_;  ///<传感器名的缓存
  std::list<RangeData> range_data_list_;
  double clear_time_ = 10.0;

  std::recursive_mutex mutex_;
};
}  // namespace CVTE_BABOT

#endif
