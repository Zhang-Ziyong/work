#include "state/elevator_done_state.hpp"
#include "robot_info.hpp"
#include <glog/logging.h>

namespace CVTE_BABOT {

std::shared_ptr<ElevatorDoneState> ElevatorDoneState::ptr_done_state_ = nullptr;

ElevatorDoneState::ElevatorDoneState() {
  follow_state_ = PathFollowerStates::ELEVATOR_DONE;
}

std::shared_ptr<ElevatorDoneState> ElevatorDoneState::getPtrInstance() {
  if (ptr_done_state_.get() == nullptr) {
    ptr_done_state_.reset(new ElevatorDoneState());
  }
  return ptr_done_state_;
}

FollowerStateRes ElevatorDoneState::dealState(PathFollower *) {
  LOG(INFO) << " ^^^^^^^^^^^^^^^^^^^^Eleavtor Done State ^^^^^^^^^^^^^^^^^^^^ ";
  RobotInfo::getPtrInstance()->setRobotMotionState(ROBOTMOTIONSTATE::STOP);
  FollowerStateRes follower_state;
  follower_state.states = getState();
  follower_state.msg = "";
  return follower_state;
}

}  // namespace CVTE_BABOT