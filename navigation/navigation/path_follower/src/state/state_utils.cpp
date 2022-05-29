/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file state_utils.cpp
 *
 *@brief cpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.3.0
 *@data 2020-09-24
 ************************************************************************/
#include "state/state_utils.hpp"
#include "state/avoidcar_state.hpp"
#include "state/done_state.hpp"
#include "state/free_state.hpp"
#include "state/local_state.hpp"
#include "state/pause_state.hpp"
#include "state/refer_state.hpp"
#include "state/rotate_state.hpp"
#include "state/recover_state.hpp"
#include "state/pit_state.hpp"

namespace CVTE_BABOT {
std::shared_ptr<State> getStateInstance(const PathFollowerStates &state) {
  // TODO: 这样枚举情况来恢复，不优雅要修改
  switch (state) {
      // case PathFollowerStates::AVOID_CAR:
      //   return AvoidCarState::getPtrInstance();
      //   break;

    case PathFollowerStates::FOLLOW_REFERENCE:
      return ReferState::getPtrInstance();
      break;

    case PathFollowerStates::FOLLOW_LOCAL:
      return LocalState::getPtrInstance();
      break;

    case PathFollowerStates::FREE:
      return FreeState::getPtrInstance();
      break;

    case PathFollowerStates::DONE:
      return DoneState::getPtrInstance();
      break;

    case PathFollowerStates::PAUSE:
      return PauseState::getPtrInstance();
      break;

    case PathFollowerStates::ROTATE:
      return RotateState::getPtrInstance();
      break;

    case PathFollowerStates::RECOVER:
      return RecoverState::getPtrInstance();
      break;

    case PathFollowerStates::PASS_PIT:
      return PitState::getPtrInstance();
      break;

    default:
      return FreeState::getPtrInstance();
      break;
  }
}

}  // namespace CVTE_BABOT
