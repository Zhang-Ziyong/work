/*
 * @Author: your name
 * @Date: 2021-06-01 16:49:38
 * @LastEditTime: 2021-06-09 11:13:15
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath:
 * /navigation/navigation/speed_decision/src/speed_decision_adapter.hpp
 */
#include "speed_decision_adapter.hpp"
namespace CVTE_BABOT {
SpeedDecisionAdapter::SpeedDecisionAdapter() {
  ptr_speed_dec_adapter_ros2_ = std::make_shared<SpeedDecisionAdapterRos2>();
  ptr_speed_decision_ = SpeedDecisionBase::getInstance();
  ptr_speed_dec_adapter_ros2_->registerScanCallback(std::bind(
      &SpeedDecisionAdapter::scanCallback, this, std::placeholders::_1));
}
void SpeedDecisionAdapter::start() {
  ptr_speed_dec_adapter_ros2_->init();
  readParams();
  ptr_speed_decision_->setConfig();
}

void SpeedDecisionAdapter::readParams() {
  ptr_navigation_mediator_ = NavigationMediator::getPtrInstance();
}

void SpeedDecisionAdapter::stop() {
  ptr_speed_dec_adapter_ros2_->stop();
}
void SpeedDecisionAdapter::spin() {
  ptr_speed_dec_adapter_ros2_->spin();
}
void SpeedDecisionAdapter::scanCallback(
    const std::vector<Eigen::Vector2d> &scan) {
  ptr_speed_decision_->inputScan(scan);
  ptr_speed_decision_->updateVel();
}
}  // namespace CVTE_BABOT
