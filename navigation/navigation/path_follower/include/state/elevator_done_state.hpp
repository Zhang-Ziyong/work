/**
 * @file elevator_done_state.hpp
 * @author linyanlong(linyanlong@cvte.com)
 * @brief
 * @version 0.1
 * @date 2022-04-19
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef ELEVATOR_DONE_STATE_HPP
#define ELEVATOR_DONE_STATE_HPP

#include "type.hpp"
#include "state.hpp"
#include <mutex>

namespace CVTE_BABOT {

class ElevatorDoneState : public State {
 public:
  ~ElevatorDoneState() = default;
  static std::shared_ptr<ElevatorDoneState> getPtrInstance();

  FollowerStateRes dealState(PathFollower *) final;

 private:
  ElevatorDoneState();
  ElevatorDoneState(const ElevatorDoneState &obj) = delete;
  ElevatorDoneState &operator=(const ElevatorDoneState &obj) = delete;

  static std::shared_ptr<ElevatorDoneState> ptr_done_state_;
};
}  // namespace CVTE_BABOT

#endif  // end of __DONE_STATE_HPP