/*
 * @Author: your name
 * @Date: 2021-06-01 16:49:07
 * @LastEditTime: 2021-06-09 11:12:27
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath:
 * /navigation/navigation/speed_decision/include/speed_decision_adapter.hpp
 */
#ifndef SPEED_DECISION_ADAPTER_HPP_
#define SPEED_DECISION_ADAPTER_HPP_
#include "speed_decision_adapter_ros2.hpp"
#include "navigation_mediator.hpp"
#include "speed_decision_config.hpp"
#include "speed_decision_base.hpp"
#include <memory>
namespace CVTE_BABOT {
class SpeedDecisionAdapter {
 public:
  SpeedDecisionAdapter();
  SpeedDecisionAdapter(const SpeedDecisionAdapter &obj) = delete;
  SpeedDecisionAdapter &operator=(const SpeedDecisionAdapter &obj) = delete;
  void start();
  void stop();
  void spin();
  void readParams();

 private:
  std::shared_ptr<SpeedDecisionAdapterRos2> ptr_speed_dec_adapter_ros2_;
  std::shared_ptr<SpeedDecisionBase> ptr_speed_decision_;
  std::shared_ptr<NavigationMediator> ptr_navigation_mediator_ = nullptr;
  void scanCallback(const std::vector<Eigen::Vector2d> &scan);
  SpeedDecisionBaseConfig speed_config_;
};
}  // namespace CVTE_BABOT

#endif